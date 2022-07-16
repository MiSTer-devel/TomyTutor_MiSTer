derive_pll_clocks
derive_clock_uncertainty

# core specific constraints
create_generated_clock -name {emu:emu|tutor:console|TMS99095:cpu|CLKOUT} -source {emu|pll|pll_inst|altera_pll_i|general[2].gpll~PLL_OUTPUT_COUNTER|divclk} -divide_by 4 -multiply_by 1 -duty_cycle 50.00 { emu:emu|tutor:console|TMS99095:cpu|CLKOUT }
