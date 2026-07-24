// Copyright 2026 Patchwing Authors. All rights reserved.
// Use of this source code is governed by a BSD-style license that can be
// found in the LICENSE file.
//
// [patchwing] iOS 混合模式运行时核心实现。设计见 patchwing_runtime.h 头注释。

#include "vm/patchwing/patchwing_runtime.h"

#include <algorithm>
#include <cstring>
#include <unordered_map>
#include <vector>

#include "platform/assert.h"
#include "vm/code_patcher.h"
#include "vm/dispatch_table.h"
#include "vm/heap/heap.h"
#include "vm/heap/safepoint.h"
#include "vm/image_snapshot.h"
#include "vm/isolate.h"
#include "vm/log.h"
#include "vm/object.h"
#include "vm/object_store.h"
#include "vm/stub_code.h"
#include "vm/stack_frame.h"
#include "vm/thread.h"

#if defined(DART_INCLUDE_SIMULATOR)
#include "vm/simulator_arm64.h"
#endif

namespace dart {
namespace patchwing {

// entry frame 布局必须与栈遍历器（SetupNextExitFrameData 的
// kExitLinkSlotFromEntryFp）一致。
static_assert(offsetof(MixedModeEntryFrame, saved_fp) -
                      offsetof(MixedModeEntryFrame, exit_link) ==
                  -kExitLinkSlotFromEntryFp * kWordSize,
              "MixedModeEntryFrame layout mismatch");

namespace {

// ---------------------------------------------------------------------------
// 全局状态
// ---------------------------------------------------------------------------

struct State {
  // base App.framework snapshot 映射（engine 在 isolate 创建前提供）。
  const uint8_t* base_isolate_data = nullptr;
  const uint8_t* base_isolate_instrs = nullptr;
  const uint8_t* base_vm_data = nullptr;
  const uint8_t* base_vm_instrs = nullptr;

  // base isolate 指令 image 区间 [start, end)。
  uword base_text_start = 0;
  uword base_text_end = 0;

  // LinkTable（从 vmcode 拷贝，进程生命周期）。
  LinkEntry* link_entries = nullptr;
  uint32_t link_count = 0;

  // patch isolate 指令 image 区间（OnIsolateSnapshotLoaded 时填入）。
  uword patch_text_start = 0;
  uword patch_text_end = 0;

  // 蹦床解析用映射：
  //   Code 地址（untag）→ 原 patch 侧 entry（改写前的 entry point）。
  std::unordered_map<uword, uword> sim_entry_by_code;
  //   dispatch table 字节偏移（相对 x21/ArrayOrigin）→ patch Code（untag）。
  std::unordered_map<uword, uword> code_by_dt_offset;

  bool tables_ready = false;   // SetupLinkTables 完成
  bool routing_ready = false;  // OnIsolateSnapshotLoaded 完成
};

State& S() {
  static State* state = new State();
  return *state;
}

// 从 instructions image 头解析指令区间 [start, end)。
// 与 InstructionsTable 的 start_pc/end_pc 同约定（image 起点 ~ image 末尾）。
void TextRangeOfImage(const uint8_t* image, uword* start, uword* end) {
  const uword raw = reinterpret_cast<uword>(image);
  const Image img(image);
  *start = raw;
  // end = object_start + object_size（== image 起点 + snapshot 总长）。
  *end = reinterpret_cast<uword>(img.object_start()) + img.object_size();
}

#if defined(DART_PRECOMPILED_RUNTIME)
// stack map payload 字节长（含 4B 头）：flags_and_size 的高 30 位为
// 长度（与 raw_object.h 的 SizeField（BitField, pos=2）一致，保持同步）。
inline size_t StackMapPayloadBytes(
    const UntaggedCompressedStackMaps::Payload* payload) {
  return sizeof(UntaggedCompressedStackMaps::Payload::FlagsAndSizeHeader) +
         (payload->flags_and_size() >> 2);
}

// Code 的 4 个 entry 字段改写。new_payload_entry_base 为新 payload 基址
// （未变函数 = base_text + cpuOffset；变更函数 = 蹦床地址，delta 取 0）。
void RewriteCodeEntries(const Code& code, uword new_payload_entry_base) {
  const uword old_payload = Code::PayloadStartOf(code.ptr());
  // 各 entry kind 相对 payload 的 delta 在 base/patch 两侧一致（未变函数
  // 机器码逐字节相同）。
  code.PatchwingSetEntryPoints(
      new_payload_entry_base + (code.EntryPoint() - old_payload),
      new_payload_entry_base + (code.UncheckedEntryPoint() - old_payload),
      new_payload_entry_base + (code.MonomorphicEntryPoint() - old_payload),
      new_payload_entry_base +
          (code.MonomorphicUncheckedEntryPoint() - old_payload));
}

// Function 对象缓存的 entry 同步（反序列化时按原 entry 填充，路由改写后
// 必须同步，否则经 Function 缓存的调用路径会跳进 patch 数据页）。
void RewriteFunctionEntryCache(const Function& function, const Code& code) {
  function.PatchwingSetEntryCache(code.EntryPoint(),
                                  code.UncheckedEntryPoint());
}
#endif  // defined(DART_PRECOMPILED_RUNTIME)

}  // namespace

bool IsActive() {
  return S().tables_ready;
}

bool IsPatchPc(uword pc) {
  return S().routing_ready && pc >= S().patch_text_start &&
         pc < S().patch_text_end;
}

bool IsBasePc(uword pc) {
  return S().routing_ready && pc >= S().base_text_start &&
         pc < S().base_text_end;
}

bool LookupLink(uint32_t sim_offset, uint32_t* cpu_offset) {
  const State& s = S();
  uint32_t lo = 0, hi = s.link_count;
  while (lo < hi) {
    const uint32_t mid = lo + (hi - lo) / 2;
    const uint32_t v = s.link_entries[mid].sim_offset;
    if (v == sim_offset) {
      *cpu_offset = s.link_entries[mid].cpu_offset;
      return true;
    } else if (v < sim_offset) {
      lo = mid + 1;
    } else {
      hi = mid;
    }
  }
  return false;
}

bool ShouldHandleInSimulator(uword pc) {
  return IsPatchPc(pc);
}

// ---------------------------------------------------------------------------
// isolate snapshot 加载完成后的路由改写
// ---------------------------------------------------------------------------

#if defined(DART_PRECOMPILED_RUNTIME)
namespace {

void SetupRouting(Thread* thread, const uint8_t* instructions_image) {
  State& s = S();
  auto zone = thread->zone();
  auto isolate_group = thread->isolate_group();
  ObjectStore* object_store = isolate_group->object_store();

  TextRangeOfImage(instructions_image, &s.patch_text_start, &s.patch_text_end);

  const uword trampoline = StubCode::PatchwingInvokeSimulator().EntryPoint();

  // patch 的第一张 InstructionsTable（root loading unit）。
  const auto& tables = GrowableObjectArray::Handle(
      zone, object_store->instructions_tables());
  RELEASE_ASSERT(!tables.IsNull() && tables.Length() > 0);
  const auto& patch_table =
      InstructionsTable::Handle(zone, InstructionsTable::RawCast(tables.At(0)));
  const Array& code_objects = Array::Handle(zone, patch_table.code_objects());
  const auto rodata = patch_table.rodata();
  const intptr_t first_with_code = patch_table.FirstEntryWithCode();
  const intptr_t count = code_objects.Length();

  Code& code = Code::Handle(zone);
  Function& owner = Function::Handle(zone);

  // 追加表条目收集（按 cpu_offset 排序后再建表）。
  struct BaseEntry {
    uint32_t cpu_offset;
    CodePtr patch_code;
    uint32_t patch_stack_map_offset;  // 在 patch rodata 中的 stack map 偏移
  };
  std::vector<BaseEntry> base_entries;

  intptr_t linked = 0, simulated = 0;
  for (intptr_t i = 0; i < count; i++) {
    code ^= code_objects.At(i);
    ASSERT(!code.IsNull());
    const uword patch_entry = Code::PayloadStartOf(code.ptr());
    RELEASE_ASSERT(patch_entry >= s.patch_text_start &&
                   patch_entry < s.patch_text_end);
    const uint32_t sim_offset =
        static_cast<uint32_t>(patch_entry - s.patch_text_start);

    uint32_t cpu_offset = 0;
    if (LookupLink(sim_offset, &cpu_offset)) {
      // 未变函数：跳 base 原生代码。
      RewriteCodeEntries(code, s.base_text_start + cpu_offset);
      base_entries.push_back(
          {cpu_offset, code.ptr(),
           rodata->entries()[first_with_code + i].stack_map_offset});
      linked++;
    } else {
      // 变更/新增函数：进模拟器。先登记原 entry（蹦床解析用），再改写。
      const uword original_entry = code.EntryPoint();
      s.sim_entry_by_code[UntaggedObject::ToAddr(code.ptr())] =
          original_entry;
      RewriteCodeEntries(code, trampoline);
      simulated++;
    }

    // 同步 Function 缓存的 entry。
    if (code.owner()->GetClassId() == kFunctionCid) {
      owner = Function::RawCast(code.owner());
      RewriteFunctionEntryCache(owner, code);
    }
  }

  // ---- dispatch table 改写 + dt 偏移 → Code 映射 --------------------------
  DispatchTable* dt = thread->isolate_group()->dispatch_table();
  if (dt != nullptr) {
    uword* dt_array = dt->MutableArray();
    const intptr_t dt_length = dt->length();
    for (intptr_t i = 0; i < dt_length; i++) {
      const uword entry = dt_array[i];
      if (entry < s.patch_text_start || entry >= s.patch_text_end) {
        continue;  // 非 patch 代码（vm stub 或空槽）跳过
      }
      // 找该 entry 对应的 patch Code（经原 patch InstructionsTable）。
      CodePtr target =
          InstructionsTable::FindCode(patch_table.ptr(), entry);
      if (target == Code::null()) {
        continue;
      }
      code ^= target;
      // 登记蹦床解析映射：x21 = ArrayOrigin()，槽位字节偏移 =
      // (i - kOriginElement) * kWordSize。
      const uword byte_offset =
          static_cast<uword>(i - DispatchTable::kOriginElement) * kWordSize;
      s.code_by_dt_offset[byte_offset] = UntaggedObject::ToAddr(code.ptr());
      // DT entry 改为该 Code 改写后的 entry。
      dt_array[i] = code.EntryPoint();
    }
  }

  // ---- 为 base text 构建追加 InstructionsTable ----------------------------
  if (!base_entries.empty()) {
    std::sort(base_entries.begin(), base_entries.end(),
              [](const BaseEntry& a, const BaseEntry& b) {
                return a.cpu_offset < b.cpu_offset;
              });

    // 布局类型经 decltype 取得（UntaggedInstructionsTable::Data 是
    // protected 嵌套类型，经公开访问器 rodata() 使用）。
    using TableData =
        std::remove_cv_t<std::remove_pointer_t<decltype(patch_table.rodata())>>;
    using DataEntry =
        std::remove_cv_t<std::remove_pointer_t<decltype(rodata->entries())>>;
    const intptr_t n = static_cast<intptr_t>(base_entries.size());
    const intptr_t entry_count = n + 1;  // +1 哨兵（pc_offset=0）

    // 先算 stack map 拷贝总量。
    size_t stack_maps_bytes = 0;
    for (const auto& e : base_entries) {
      stack_maps_bytes +=
          StackMapPayloadBytes(rodata->StackMapAt(e.patch_stack_map_offset));
    }

    const size_t rodata_size =
        sizeof(TableData) + entry_count * sizeof(DataEntry) + stack_maps_bytes;
    uint8_t* new_rodata = static_cast<uint8_t*>(malloc(rodata_size));
    RELEASE_ASSERT(new_rodata != nullptr);

    TableData* data = reinterpret_cast<TableData*>(new_rodata);
    data->canonical_stack_map_entries_offset =
        rodata->canonical_stack_map_entries_offset;
    data->length = static_cast<uint32_t>(entry_count);
    data->first_entry_with_code = 1;
    data->padding = 0;

    DataEntry* entries = reinterpret_cast<DataEntry*>(new_rodata +
                                                      sizeof(TableData));
    uint8_t* maps_out = new_rodata + sizeof(TableData) +
                        entry_count * sizeof(DataEntry);

    // 哨兵：pc_offset=0，stack map 借用第一个真实条目的拷贝（该区间
    // 无代码，实际不可达）。
    entries[0].pc_offset = 0;
    entries[0].stack_map_offset = static_cast<uint32_t>(maps_out - new_rodata);

    const auto& base_table = InstructionsTable::Handle(
        zone, InstructionsTable::New(entry_count, s.base_text_start,
                                     s.base_text_end,
                                     reinterpret_cast<uword>(new_rodata)));

    for (intptr_t i = 0; i < n; i++) {
      const auto& e = base_entries[i];
      entries[i + 1].pc_offset = e.cpu_offset;
      entries[i + 1].stack_map_offset =
          static_cast<uint32_t>(maps_out - new_rodata);
      const auto* payload = rodata->StackMapAt(e.patch_stack_map_offset);
      const size_t payload_size = StackMapPayloadBytes(payload);
      memcpy(maps_out, payload, payload_size);
      maps_out += payload_size;
      base_table.SetCodeAt(i, e.patch_code);
    }
    RELEASE_ASSERT(maps_out == new_rodata + rodata_size);

    tables.Add(base_table, Heap::kOld);
  }

  s.routing_ready = true;

  THR_Print("[patchwing] mixed mode active: linked=%" Pd
            " simulated=%" Pd " base_text=[%" Px ",%" Px ") patch_text=[%" Px
            ",%" Px ")\n",
            linked, simulated, s.base_text_start, s.base_text_end,
            s.patch_text_start, s.patch_text_end);
}

}  // namespace
#endif  // defined(DART_PRECOMPILED_RUNTIME)

void OnIsolateSnapshotLoaded(Thread* thread, const uint8_t* instructions_image) {
#if defined(DART_PRECOMPILED_RUNTIME)
  if (!IsActive()) {
    return;
  }
  ASSERT(!S().routing_ready);  // 单 isolate 假设（v1）
  SetupRouting(thread, instructions_image);

#if defined(DART_INCLUDE_SIMULATOR)
  // per-thread trampoline 上下文缓冲 + 蹦床 impl 函数指针（trampoline
  // stub 经 THR 偏移寻址，必须提前填充）。
  if (thread->patchwing_context() == nullptr) {
    thread->set_patchwing_context(new CpuContext());
  }
  thread->set_patchwing_invoke_impl(
      reinterpret_cast<uword>(&InvokeSimulatorImpl));
#endif
#else
  USE(thread);
  USE(instructions_image);
#endif  // defined(DART_PRECOMPILED_RUNTIME)
}

// ---------------------------------------------------------------------------
// 蹦床 C++ 入口（native→sim）
// ---------------------------------------------------------------------------

#if defined(DART_PRECOMPILED_RUNTIME)
namespace {

#if defined(DART_INCLUDE_SIMULATOR)

// 校验并解码 pool 静态调用点（AOT BranchLink）：
//   ldr LR, [PP, #off] （或 add+ldr 两条）; ldr LR, [LR, #entry_off]; blr LR
// 命中返回目标 Code，未命中返回 null。
// 注：base/patch 调用方共享同一全局池（布局兼容），直接用当前全局池解码。
CodePtr ResolvePoolCallTarget(Thread* thread, uword return_pc) {
  auto zone = thread->zone();

  const uint32_t blr_lr = *reinterpret_cast<const uint32_t*>(return_pc) - 1;
  if (blr_lr != 0xd63f03c0u) {  // blr x30
    return Code::null();
  }

  // 解码 pc-8 的池加载：ldr Xt, [PP, #imm]（或 add Xt,PP,#hi + ldr Xt,[Xt,#lo]）。
  const uint32_t ldr = *(reinterpret_cast<const uint32_t*>(return_pc) - 2);
  // LoadStoreRegOp && bit22(==1 load) && bits(30,2)==3(64-bit) && bit24(==1
  // unsigned imm)
  const bool is_ldr = ((ldr >> 25) & 0x7) == 0x5 && ((ldr >> 22) & 1) == 1 &&
                      ((ldr >> 30) & 0x3) == 0x3 && ((ldr >> 24) & 1) == 1;
  if (!is_ldr) {
    return Code::null();
  }
  const uint32_t rt = ldr & 0x1f;
  const uint32_t rn = (ldr >> 5) & 0x1f;
  uint32_t offset = ((ldr >> 10) & 0xfff) << 3;
  if (rn != 27 /* PP */) {
    if (rn != rt) {
      return Code::null();
    }
    // 两条形式：前一条必须是 add Xt, PP, #hi12。
    const uint32_t add = *(reinterpret_cast<const uint32_t*>(return_pc) - 3);
    const bool is_add = ((add >> 24) & 0x9f) == 0x11 &&  // add (imm), 64-bit
                        ((add >> 5) & 0x1f) == 27 && (add & 0x1f) == rt;
    if (!is_add) {
      return Code::null();
    }
    offset |= ((add >> 10) & 0xfff) << 12;
  }
  if (rt != 30 /* LR */) {
    return Code::null();
  }

  const ObjectPool& pool =
      ObjectPool::Handle(zone, thread->global_object_pool());
  if (pool.IsNull()) {
    return Code::null();
  }
  const intptr_t index =
      (static_cast<intptr_t>(offset) - ObjectPool::element_offset(0)) /
      kWordSize;
  if (index < 0 || index >= pool.Length()) {
    return Code::null();
  }
  const Object& target = Object::Handle(zone, pool.ObjectAt(index));
  if (!target.IsCode()) {
    return Code::null();
  }
  return Code::Cast(target).ptr();
}

// 解码 dispatch table 调用点：ldr Xt, [x21, #imm]; ...; blr。
CodePtr ResolveDispatchCallTarget(Thread* thread, uword return_pc) {
  const uint32_t ldr = *(reinterpret_cast<const uint32_t*>(return_pc) - 2);
  const bool is_ldr = ((ldr >> 25) & 0x7) == 0x5 && ((ldr >> 22) & 1) == 1 &&
                      ((ldr >> 30) & 0x3) == 0x3 && ((ldr >> 24) & 1) == 1;
  if (!is_ldr) {
    return Code::null();
  }
  const uint32_t rn = (ldr >> 5) & 0x1f;
  if (rn != 21 /* DISPATCH_TABLE_REG */) {
    return Code::null();
  }
  const uword byte_offset = ((ldr >> 10) & 0xfff) << 3;
  const auto it = S().code_by_dt_offset.find(byte_offset);
  if (it == S().code_by_dt_offset.end()) {
    return Code::null();
  }
  return Code::RawCast(UntaggedObject::FromAddr(it->second));
}

// 经 x0 的解析：VM 直接调用（DartEntry）或闭包调用。
CodePtr ResolveByFirstArgument(Thread* thread, uword x0_value) {
  auto zone = thread->zone();
  if (x0_value == 0 || (x0_value & kHeapObjectTag) == 0) {
    return Code::null();  // Smi 或空，不可能是 Function/Closure
  }
  const Object& obj = Object::Handle(
      zone, UntaggedObject::FromAddr(x0_value - kHeapObjectTag));
  Function& function = Function::Handle(zone);
  if (obj.IsClosure()) {
    function ^= Closure::Cast(obj).function();
  } else if (obj.GetClassId() == kFunctionCid) {
    function ^= obj.ptr();
  } else {
    return Code::null();
  }
  if (function.IsNull() || !function.HasCode()) {
    return Code::null();
  }
  return function.CurrentCode();
}

CodePtr ResolveTargetCode(Thread* thread, const CpuContext* ctx) {
  const uword return_pc = ctx->x[30];

  // 调用点在代码区（base/patch/vm snapshot text）→ 解码调用序列。
  Code& target = Code::Handle(thread->zone());
  target = ResolvePoolCallTarget(thread, return_pc);
  if (target.IsNull()) {
    target = ResolveDispatchCallTarget(thread, return_pc);
  }
  if (target.IsNull()) {
    target = ResolveByFirstArgument(thread, ctx->x[0]);
  }

  if (target.IsNull()) {
    FATAL("[patchwing] failed to resolve mixed-mode call target (pc=%" Px
          ", x0=%" Px ")\n",
          return_pc, ctx->x[0]);
  }
  return target.ptr();
}

#endif  // defined(DART_INCLUDE_SIMULATOR)

}  // namespace

void InvokeSimulatorImpl(Thread* thread, CpuContext* ctx) {
#if defined(DART_INCLUDE_SIMULATOR)
  CodePtr target_code;
  uword sim_entry;
  uword return_pc;
  {
    // 解析阶段：可能触发句柄分配，转入 VM 状态。
    TransitionGeneratedToVM transition(thread);
    target_code = ResolveTargetCode(thread, ctx);
    const uword code_addr = UntaggedObject::ToAddr(target_code);
    const auto it = S().sim_entry_by_code.find(code_addr);
    if (it == S().sim_entry_by_code.end()) {
      FATAL("[patchwing] resolved code %p has no sim entry\n",
            reinterpret_cast<void*>(code_addr));
    }
    sim_entry = it->second;

    // 本层 return_pc（sim 函数返回检测 + 调用方帧 pc 正确性）：
    // - B-case（native 直接调用）：ctx->x[30] = 原生返回地址（base/patch
    //   text 内的调用点地址）。
    // - A-case（sim 经 callout 到蹦床）：ctx->x[30] 是 callout stub 内的
    //   返回地址（对 sim 无意义）；真正的 sim 返回地址由 DoMixedModeCallout
    //   经 Thread::patchwing_pending_sim_lr_ 桥接过来。
    return_pc = ctx->x[30];
    if (thread->patchwing_pending_sim_lr() != 0) {
      return_pc = thread->patchwing_pending_sim_lr();
      thread->set_patchwing_pending_sim_lr(0);
    }
  }

  // 构建 sim 侧 entry frame（C++ 局部，Execute 存续期间有效）：
  // [entry_fp]=0（entry 标记），[entry_fp-184]=exit-link → 来源侧锚点
  // （当前 top_exit，即蹦床 stub 帧 fp）。
  MixedModeEntryFrame entry_frame = {};
  entry_frame.exit_link = thread->top_exit_frame_info();
  entry_frame.saved_fp = 0;
  entry_frame.saved_pc = return_pc;
  const uword entry_fp =
      reinterpret_cast<uword>(&entry_frame.saved_fp);

  // 执行阶段：与原生 Dart 执行同等状态（kThreadInGenerated + top_exit=0）。
  // GC/安全点依赖 sim 代码内的原生轮询经 callout 进 runtime 处理。
  thread->set_top_exit_frame_info(0);

  Simulator* sim = Simulator::Current();
  sim->MixedModeExecute(sim_entry, return_pc, entry_fp, ctx);

  // MixedModeExecute 已把结果写回 ctx。恢复 exit frame 链。
  thread->set_top_exit_frame_info(entry_frame.exit_link);
#else
  UNREACHABLE();
#endif
}
#endif  // defined(DART_PRECOMPILED_RUNTIME)

}  // namespace patchwing
}  // namespace dart

// ---------------------------------------------------------------------------
// engine 契约 API（extern "C"）
// ---------------------------------------------------------------------------

extern "C" int Patchwing_ReadLinkHeader(const uint8_t* data, size_t size) {
  using namespace dart::patchwing;
  if (data == nullptr || size < 36) {
    return -1;
  }
  if (memcmp(data, VmcodeHeader::kMagic, 8) != 0) {
    return -2;
  }
  uint32_t version, header_size, elf_offset;
  memcpy(&version, data + 8, 4);
  memcpy(&header_size, data + 12, 4);
  memcpy(&elf_offset, data + 16, 4);
  if (version != VmcodeHeader::kVersion || header_size < 36 ||
      elf_offset < header_size || elf_offset >= size) {
    return -3;
  }
  return static_cast<int>(elf_offset);
}

extern "C" void Patchwing_SetBaseSnapshots(
    const uint8_t* isolate_snapshot_data,
    const uint8_t* isolate_snapshot_instructions,
    const uint8_t* vm_snapshot_data,
    const uint8_t* vm_snapshot_instructions) {
  auto& s = dart::patchwing::S();
  s.base_isolate_data = isolate_snapshot_data;
  s.base_isolate_instrs = isolate_snapshot_instructions;
  s.base_vm_data = vm_snapshot_data;
  s.base_vm_instrs = vm_snapshot_instructions;
  dart::patchwing::TextRangeOfImage(isolate_snapshot_instructions,
                                    &s.base_text_start, &s.base_text_end);
}

extern "C" int Patchwing_SetupLinkTables(const uint8_t* data, size_t size) {
  using namespace dart::patchwing;
  if (data == nullptr || size < 36 ||
      memcmp(data, VmcodeHeader::kMagic, 8) != 0) {
    return -1;
  }
  uint32_t link_off, link_count;
  memcpy(&link_off, data + 20, 4);
  memcpy(&link_count, data + 24, 4);
  const size_t need = static_cast<size_t>(link_off) +
                      static_cast<size_t>(link_count) * sizeof(LinkEntry);
  if (need > size) {
    return -2;
  }
  auto& s = S();
  if (s.link_entries != nullptr) {
    free(s.link_entries);
    s.link_entries = nullptr;
  }
  s.link_count = link_count;
  if (link_count > 0) {
    s.link_entries = static_cast<LinkEntry*>(
        malloc(link_count * sizeof(LinkEntry)));
    if (s.link_entries == nullptr) {
      return -3;
    }
    memcpy(s.link_entries, data + link_off,
           link_count * sizeof(LinkEntry));
  }
  s.tables_ready = true;
  return 0;
}
