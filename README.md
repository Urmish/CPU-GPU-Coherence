This repository implements a region based coherence protocol for fused CPU-GPU systems.

cd gem5/

scons build/X86_VI_hammer_GPU/gem5.opt --default=X86 EXTRAS=../gem5-gpu/src:../gpgpu-sim/ PROTOCOL=VI_hammer

gem5/build.log - last build log

Files to modified -

1. gem5-gpu/src/mem/protocol/VI_hammer-GPUL2cache.sm
1. gem5-gpu/src/mem/protocol/VI_hammer-dir.sm
1. gem5-gpu/src/mem/protocol/VI_hammer-CPUCache.sm
1. gem5/src/mem/slicc/symbols/*

File added -

gem5/src/mem/ruby/structures/RegionBuffer.*

