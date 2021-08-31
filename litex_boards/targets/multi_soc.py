#!/usr/bin/env python3

#
# This file is part of LiteX-Boards.
#
# Copyright (c) 2019 msloniewski <marcin.sloniewski@gmail.com>
# Modified 2021 by mpelcat <mpelcat@insa-rennes.fr>
# SPDX-License-Identifier: BSD-2-Clause

import os
import argparse

from migen import *
from migen.genlib.resetsync import AsyncResetSynchronizer

from litex_boards.platforms import terasic_de10lite_multisoc # referencing the platform

from litex.soc.cores.clock import Max10PLL
from litex.soc.integration.soc import SoCRegion
from litex.soc.integration.soc_core import *
from litex.soc.integration.builder import *

class _CRG(Module): # Clock Region definition
    def __init__(self, platform, sys_clk_freq):
        self.rst = Signal()
        self.clock_domains.cd_sys = ClockDomain()

        clk50 = platform.request("clk50")

        # PLL - instanciating an Intel FPGA PLL outputing a clock at sys_clk_freq
        self.submodules.pll = pll = Max10PLL(speedgrade="-7")
        self.comb += pll.reset.eq(self.rst)
        pll.register_clkin(clk50, 50e6)
        pll.create_clkout(self.cd_sys, sys_clk_freq)

class BaseSoC(SoCCore): # SoC definition - memory sizes are overloaded
    def __init__(self, sys_clk_freq=int(50e6), with_video_terminal=False, **kwargs):
        platform = terasic_de10lite_multisoc.Platform()

        # SoCCore ----------------------------------------------------------------------------------
        #These kwargs overwrite the value find on soc / soc_core
        #So you can change here the sizes of the different memories
        kwargs["integrated_rom_size"] = 0x8000 # chose rom size, holding bootloader (min = 0x6000)
        kwargs["integrated_sram_size"] = 0x8000 # chose sram size, holding stack and heap. (min = 0x6000)
        kwargs["integrated_main_ram_size"] = 0x8000 # Main RAM is external RAM. 0 means only internal BRAM memory
	
        SoCCore.__init__(self, platform, sys_clk_freq,
            ident          = "LiteX SoC on DE10-Lite",
            ident_version  = True,
            **kwargs)

        self.submodules.crg = _CRG(platform, sys_clk_freq) # CRG instanciation

#class SuperSoc(Module)
#  def __init__(self):
#    soc0
    
def main(): # Instanciating the SoC and options
    parser = argparse.ArgumentParser(description="LiteX SoC on DE10-Lite")
    parser.add_argument("--build",               action="store_true", help="Build bitstream")
    parser.add_argument("--load",                action="store_true", help="Load bitstream")
    parser.add_argument("--sys-clk-freq",        default=50e6,        help="System clock frequency (default: 50MHz)")
    parser.add_argument("--with-video-terminal", action="store_true", help="Enable Video Terminal (VGA)")
    builder_args(parser)
    soc_core_args(parser)
    args = parser.parse_args()
    
    # Building and generating a list of complete SoCs
    socs = []
    
    # Setting up SoCs sequentially with custom UART signals
    for i in range(0,1):
      kwargs = soc_core_argdict(args)
      kwargs["uart_name"] = "serial" + str(i) # indexing each serial port with soc index i
      soc = BaseSoC(
        sys_clk_freq        = int(float(args.sys_clk_freq)),
        with_video_terminal = args.with_video_terminal,
        **kwargs
      )
      socs.append(soc)
    
      # Generating each soc in a distinct directory
      kwargs = builder_argdict(args)
      kwargs["output_dir"] = os.path.join("build", soc.platform.name,"soc" + str(i))
      builder = Builder(soc, **kwargs)
      builder.build(run=args.build)

    
    
    if args.load:
        prog = soc[0].platform.create_programmer()
        prog.load_bitstream(os.path.join(builder.gateware_dir, soc[0].build_name + ".sof"))

if __name__ == "__main__":
    main()
