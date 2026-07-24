// Copyright 2026 Patchwing Authors. All rights reserved.
// Use of this source code is governed by a BSD-style license that can be
// found in the LICENSE file.
//
// [patchwing] iOS 混合模式（mixed mode）运行时核心。
//
// 机制概述（详见 flutter_update/docs/IOS_MIXED_MODE_PLAN.md §10）：
// - patch 激活时 isolate 从 patch snapshot 启动（patch 堆/patch 全局池），
//   base App.framework 仅作为「代码库」被 dyld 映射（签名、可执行）。
// - LinkTable（aot_tools link 产出）记录未变函数 simOffset→cpuOffset 映射。
// - isolate snapshot 反序列化完成后（OnIsolateSnapshotLoaded）：
//   * 未变函数的 patch Code entry 改写为 base 原生代码地址（直跑，快）；
//   * 变更/新增函数的 patch Code entry 改写为 StubCode::PatchwingInvokeSimulator
//     （蹦床，进 SIMARM64 模拟器解释执行 patch 数据页中的代码，iOS 合规）；
//   * dispatch table 同步改写，并登记 dt slot → Code 映射供蹦床解析；
//   * 为 base text 追加构建一张 InstructionsTable（GC/异常栈遍历需要）。
// - 双向切换：native→sim 由蹦床 stub 完成；sim→native 由模拟器 Execute 循环
//   检测 pc 离开 patch text 后走 callout stub 完成。
//
// 栈纪律（GC/异常遍历正确性的关键）：
// - sim 与 native 共享真实栈（栈参数零拷贝），但两侧帧**不直接以 fp 链
//   互穿**——每次穿越都在目标侧构建标准 **entry frame**（saved-fp=0 标记 +
//   [fp-184] exit-link 槽，与 InvokeDartCode 的 entry frame 同布局），
//   exit-link 指回来源侧的锚点 fp。栈遍历经 SetupNextExitFrameData 沿
//   exit-link 跨世界推进，所有帧都以正确 pc 分类（DartFrame/StubFrame/
//   ExitFrame），避免保守扫描 sim 帧体（内含未装箱裸值，误扫必崩）。
// - sim 函数返回检测：Execute 循环比较 pc == 本层 return_pc（调用点返回
//   地址，绝不可能是合法的调用目标），命中则结束本次执行。

#ifndef RUNTIME_VM_PATCHWING_PATCHWING_RUNTIME_H_
#define RUNTIME_VM_PATCHWING_PATCHWING_RUNTIME_H_

#include <cstddef>
#include <cstdint>

#include "vm/globals.h"

namespace dart {

class Thread;

namespace patchwing {

// vmcode 文件头（PWNGVMCD）布局。与 aot_tools link 的输出契约（M2）。
// [8B magic][u32 version][u32 header_size][u32 elf_offset]
// [u32 link_off][u32 link_count][u32 reserved_x2]
struct VmcodeHeader {
  static constexpr char kMagic[8] = {'P', 'W', 'N', 'G', 'V', 'M', 'C', 'D'};
  static constexpr uint32_t kVersion = 1;
};

// LinkTable 条目：未变函数 patch 侧偏移 → base 侧偏移。
// 偏移均相对各自 instructions image 起点（与 InstructionsTable::pc_offset
// 同约定）。按 sim_offset 升序。
struct LinkEntry {
  uint32_t sim_offset;
  uint32_t cpu_offset;
};

// 双向切换时的全寄存器上下文。trampoline stub（native→sim）与
// callout stub（sim→native）共用布局；asm 侧用 offsetof 同步。
// 存放于 malloc 内存（不得在 GC 可遍历的栈帧范围内出现裸值）。
struct CpuContext {
  uword x[31];   // x0..x30（x30 = LR；trampoline 入口时由 stub 写入返回哨兵）
  uword sp;      // 调用点真实 SP（指向栈参数区）
  uword nzcv;    // 条件码
  uword pad;
  double v[16];  // v0..v15 低 64 位
};

// 混合模式是否已激活（已收到 base snapshots + link tables）。
bool IsActive();

// pc 区间判定。IsPatchPc：patch 指令页（模拟器执行区）；
// IsBasePc：base App.framework 指令页（原生执行区）。
bool IsPatchPc(uword pc);
bool IsBasePc(uword pc);

// LinkTable 查找：patch 侧偏移 → base 侧偏移。
bool LookupLink(uint32_t sim_offset, uint32_t* cpu_offset);

// app_snapshot.cc 的 ReadProgramSnapshot 在反序列化完成后调用。
// instructions_image 为本次加载的 isolate 指令 image（patch 激活时 = patch）。
void OnIsolateSnapshotLoaded(Thread* thread, const uint8_t* instructions_image);

// 异常处理路由（exceptions.cc::JumpToFrame 调用）：目标 pc 是否应由
// 模拟器接管（Simulator::JumpToFrame）。
bool ShouldHandleInSimulator(uword pc);

// 蹦床 C++ 入口（由 StubCode::PatchwingInvokeSimulator 经 CallCFunction 调用）。
// ctx 为 per-thread malloc 缓冲（Thread::patchwing_context_），内含调用点
// 全寄存器；返回前把 sim 侧结果（x0/v0）写回 ctx。
void InvokeSimulatorImpl(Thread* thread, CpuContext* ctx);

// 世界穿越时构建的 entry frame（与 InvokeDartCode 同布局：exit-link 槽
// 在 fp + kExitLinkSlotFromEntryFp*kWordSize = fp-184 处）。
// saved_fp=0 是遍历器的 entry 标记；exit_link 指向来源侧锚点帧。
// 作为 C++ 局部对象分配（在原生调用存续期间有效）。
struct MixedModeEntryFrame {
  uword pad_before;    // offset 0
  uword exit_link;     // offset 8（== saved_fp - 184）
  uword reserved[22];  // offset 16..191
  uword saved_fp;      // offset 192：必须为 0（entry 标记）
  uword saved_pc;      // offset 200：仅调试可读
};

}  // namespace patchwing
}  // namespace dart

extern "C" {

// ---- 与 engine（runtime/patchwing/patchwing_dart_stubs.h）的契约 ----

// 解析 vmcode 头，返回 ELF 在文件中的偏移；失败返回负值。
int Patchwing_ReadLinkHeader(const uint8_t* data, size_t size);

// 记录 base App.framework 的 snapshot 映射（engine 在 isolate 创建前调用）。
void Patchwing_SetBaseSnapshots(const uint8_t* isolate_snapshot_data,
                                const uint8_t* isolate_snapshot_instructions,
                                const uint8_t* vm_snapshot_data,
                                const uint8_t* vm_snapshot_instructions);

// 把 vmcode 中的 LinkTable 拷贝进 VM 全局状态（engine 在 ReadLinkHeader
// 之后、isolate 创建前调用；data 为整个 vmcode 映射）。成功返回 0。
int Patchwing_SetupLinkTables(const uint8_t* data, size_t size);

}  // extern "C"

#endif  // RUNTIME_VM_PATCHWING_PATCHWING_RUNTIME_H_
