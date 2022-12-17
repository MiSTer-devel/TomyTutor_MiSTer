//============================================================================
//
//  This program is free software; you can redistribute it and/or modify it
//  under the terms of the GNU General Public License as published by the Free
//  Software Foundation; either version 2 of the License, or (at your option)
//  any later version.
//
//  This program is distributed in the hope that it will be useful, but WITHOUT
//  ANY WARRANTY; without even the implied warranty of MERCHANTABILITY or
//  FITNESS FOR A PARTICULAR PURPOSE.  See the GNU General Public License for
//  more details.
//
//  You should have received a copy of the GNU General Public License along
//  with this program; if not, write to the Free Software Foundation, Inc.,
//  51 Franklin Street, Fifth Floor, Boston, MA 02110-1301 USA.
//
//============================================================================

module emu
(
	//Master input clock
	input         CLK_50M,

	//Async reset from top-level module.
	//Can be used as initial reset.
	input         RESET,

	//Must be passed to hps_io module
	inout  [48:0] HPS_BUS,

	//Base video clock. Usually equals to CLK_SYS.
	output        CLK_VIDEO,

	//Multiple resolutions are supported using different CE_PIXEL rates.
	//Must be based on CLK_VIDEO
	output        CE_PIXEL,

	//Video aspect ratio for HDMI. Most retro systems have ratio 4:3.
	//if VIDEO_ARX[12] or VIDEO_ARY[12] is set then [11:0] contains scaled size instead of aspect ratio.
	output [12:0] VIDEO_ARX,
	output [12:0] VIDEO_ARY,

	output  [7:0] VGA_R,
	output  [7:0] VGA_G,
	output  [7:0] VGA_B,
	output        VGA_HS,
	output        VGA_VS,
	output        VGA_DE,    // = ~(VBlank | HBlank)
	output        VGA_F1,
	output [1:0]  VGA_SL,
	output        VGA_SCALER, // Force VGA scaler

	input  [11:0] HDMI_WIDTH,
	input  [11:0] HDMI_HEIGHT,
	output        HDMI_FREEZE,

`ifdef MISTER_FB
	// Use framebuffer in DDRAM (USE_FB=1 in qsf)
	// FB_FORMAT:
	//    [2:0] : 011=8bpp(palette) 100=16bpp 101=24bpp 110=32bpp
	//    [3]   : 0=16bits 565 1=16bits 1555
	//    [4]   : 0=RGB  1=BGR (for 16/24/32 modes)
	//
	// FB_STRIDE either 0 (rounded to 256 bytes) or multiple of pixel size (in bytes)
	output        FB_EN,
	output  [4:0] FB_FORMAT,
	output [11:0] FB_WIDTH,
	output [11:0] FB_HEIGHT,
	output [31:0] FB_BASE,
	output [13:0] FB_STRIDE,
	input         FB_VBL,
	input         FB_LL,
	output        FB_FORCE_BLANK,

`ifdef MISTER_FB_PALETTE
	// Palette control for 8bit modes.
	// Ignored for other video modes.
	output        FB_PAL_CLK,
	output  [7:0] FB_PAL_ADDR,
	output [23:0] FB_PAL_DOUT,
	input  [23:0] FB_PAL_DIN,
	output        FB_PAL_WR,
`endif
`endif

	output        LED_USER,  // 1 - ON, 0 - OFF.

	// b[1]: 0 - LED status is system status OR'd with b[0]
	//       1 - LED status is controled solely by b[0]
	// hint: supply 2'b00 to let the system control the LED.
	output  [1:0] LED_POWER,
	output  [1:0] LED_DISK,

	// I/O board button press simulation (active high)
	// b[1]: user button
	// b[0]: osd button
	output  [1:0] BUTTONS,

	input         CLK_AUDIO, // 24.576 MHz
	output [15:0] AUDIO_L,
	output [15:0] AUDIO_R,
	output        AUDIO_S,   // 1 - signed audio samples, 0 - unsigned
	output  [1:0] AUDIO_MIX, // 0 - no mix, 1 - 25%, 2 - 50%, 3 - 100% (mono)

	//ADC
	inout   [3:0] ADC_BUS,

	//SD-SPI
	output        SD_SCK,
	output        SD_MOSI,
	input         SD_MISO,
	output        SD_CS,
	input         SD_CD,

	//High latency DDR3 RAM interface
	//Use for non-critical time purposes
	output        DDRAM_CLK,
	input         DDRAM_BUSY,
	output  [7:0] DDRAM_BURSTCNT,
	output [28:0] DDRAM_ADDR,
	input  [63:0] DDRAM_DOUT,
	input         DDRAM_DOUT_READY,
	output        DDRAM_RD,
	output [63:0] DDRAM_DIN,
	output  [7:0] DDRAM_BE,
	output        DDRAM_WE,

	//SDRAM interface with lower latency
	output        SDRAM_CLK,
	output        SDRAM_CKE,
	output [12:0] SDRAM_A,
	output  [1:0] SDRAM_BA,
	inout  [15:0] SDRAM_DQ,
	output        SDRAM_DQML,
	output        SDRAM_DQMH,
	output        SDRAM_nCS,
	output        SDRAM_nCAS,
	output        SDRAM_nRAS,
	output        SDRAM_nWE,

`ifdef MISTER_DUAL_SDRAM
	//Secondary SDRAM
	//Set all output SDRAM_* signals to Z ASAP if SDRAM2_EN is 0
	input         SDRAM2_EN,
	output        SDRAM2_CLK,
	output [12:0] SDRAM2_A,
	output  [1:0] SDRAM2_BA,
	inout  [15:0] SDRAM2_DQ,
	output        SDRAM2_nCS,
	output        SDRAM2_nCAS,
	output        SDRAM2_nRAS,
	output        SDRAM2_nWE,
`endif

	input         UART_CTS,
	output        UART_RTS,
	input         UART_RXD,
	output        UART_TXD,
	output        UART_DTR,
	input         UART_DSR,

	// Open-drain User port.
	// 0 - D+/RX
	// 1 - D-/TX
	// 2..6 - USR2..USR6
	// Set USER_OUT to 1 to read from USER_IN.
	input   [6:0] USER_IN,
	output  [6:0] USER_OUT,

	input         OSD_STATUS
);

///////// Default values for ports not used in this core /////////

assign ADC_BUS  = 'Z;
assign USER_OUT = '1;
assign {UART_RTS, UART_TXD, UART_DTR} = 0;
assign {SD_SCK, SD_MOSI, SD_CS} = 'Z;
assign {SDRAM_DQ, SDRAM_A, SDRAM_BA, SDRAM_CLK, SDRAM_CKE, SDRAM_DQML, SDRAM_DQMH, SDRAM_nWE, SDRAM_nCAS, SDRAM_nRAS, SDRAM_nCS} = 'Z;
assign {DDRAM_CLK, DDRAM_BURSTCNT, DDRAM_ADDR, DDRAM_DIN, DDRAM_BE, DDRAM_RD, DDRAM_WE} = '0;  

//assign VGA_SL = 0;
assign VGA_F1 = 0;
assign VGA_SCALER = 0;
assign HDMI_FREEZE = 0;

wire [10:0] audio;
assign AUDIO_L = {audio,5'd0};
assign AUDIO_R = {audio,5'd0};
assign AUDIO_S = 0;
assign AUDIO_MIX = 0;

assign LED_DISK = 0;
assign LED_POWER = 0;
assign LED_USER = 0;
assign BUTTONS = osd_btn;

////////////////////////////////////////////////////////////////////
// Status Bit Map:                                                                                     1         1         1
// 0         1         2         3         4         5         6         7         8         9         0         1         2
// 01234567890123456789012345678901234567890123456789012345678901234567890123456789012345678901234567890123456789012345678901234567
// XXXXXXXXXXXXXXXXXX

// The Dev Menu is hidden by default because most, if not all users will never need to use it.
// The Tutor/Pyuta had an expansion port that allowed for devices with ram such as Tanam1972's Rom/Ram cartridge.
// There are game roms/cart images that utilize certain banks of RAM but currently, the code can detect those and enable
// them accordingly.  This menu is meant for developers who want to write or port software that may need to use these banks.
// To enable/disable the menu and ram banks, press ALT-F11.

`include "build_id.v" 
localparam CONF_STR = {
	"TomyTutor;;",
	"FC2,BIN,Load Cartridge;",

	"P1,Tape;",
	"P1S0,CAS,Mount Tape Image;",
	"D0P1O[2],Tape Read Only,No,Yes;",
	"P1O[4:3],Tape Status,Never,Always,When Mounted, With Activity;",
	"P1O[5],Tape Audio,Muted,UnMuted;",
	"D1P1T[6},Stop   [F5];",
	"D1P1T[7],Rewind [F6];",

	"h2P2,Dev Menu;",
	"h2P2O[13],4000-5FFF Ram,Off,On;",
	"h2P2O[14],6000-7FFF Ram,Off,On;",
	"h2P2O[15],8000-9FFF Ram,Off,On;",
	"h2P2O[16],A000-BFFF Ram,Off,On;",
	"h2P2O[17],C000-DFFF Ram,Off,On;",
	"-;",

	"P3,Video Settings;",
	"P3O[8],MAX Num of Sprites,32,4;",
	"P3O[10:9],Scale,Normal,V-Integer,Narrower HV-Integer,Wider HV-Integer;",
	"P3O[11],Aspect Ratio,Original,Full Screen;",
	"P3O[12],Vertical Crop,Off,On;",
	"-;",
	"FC1,ROM,Select System Rom;",
	"-;",
   "R[1],Eject Cartridge;",
	"-;",
	"R[0],Reset;",
	"J,Fire 1,Fire 2,1,2,Enter/RT;",
	"V,v",`BUILD_DATE 
};

wire  [1:0] buttons;
wire [127:0] status;
wire [127:0] status_o;		//So we can update OCD Settings on the fly
wire [15:0] status_mask = {12'd0, dev_menu_en, ~tape_mounted, (img_readonly_r || ~tape_mounted)};
wire        status_update;

wire [31:0] joy0, joy1;
wire [10:0] ps2_key;

wire        ioctl_download;
wire  [7:0] ioctl_index;
wire        ioctl_wr;
wire [24:0] ioctl_addr;
wire  [7:0] ioctl_dout;

wire        forced_scandoubler;
wire [21:0] gamma_bus;

wire        tape_readonly = status[2];
wire        img_readonly, img_readonly_r;
wire        sd_rd;
wire        sd_wr;
wire        sd_ack;
wire [31:0] sd_lba[1];
wire  [9:0] sd_buff_addr;
wire  [7:0] sd_buff_dout;
wire  [7:0] sd_buff_din[1];
wire	[5:0] sd_blk_cnt[1];
wire        sd_buff_wr;
wire        img_mounted;
wire [31:0] img_size, tape_size;

assign sd_blk_cnt[0] = 6'd0;


hps_io #(.CONF_STR(CONF_STR), .VDNUM(1), .BLKSZ(3)) hps_io
(
	.clk_sys(clk_sys),
	.HPS_BUS(HPS_BUS),
	.EXT_BUS(),
	.gamma_bus(gamma_bus),

	.forced_scandoubler(forced_scandoubler),

	.buttons(buttons),
	.status(status),
	.status_in(status_o),
	.status_set(status_update),
	.status_menumask(status_mask),

	.ioctl_download(ioctl_download),
	.ioctl_index(ioctl_index),
	.ioctl_wr(ioctl_wr),
	.ioctl_addr(ioctl_addr),
	.ioctl_dout(ioctl_dout),


	.sd_lba(sd_lba),
	.sd_rd(sd_rd),
	.sd_wr(sd_wr),
	.sd_ack(sd_ack),
	.sd_buff_addr(sd_buff_addr),
	.sd_buff_dout(sd_buff_dout),
	.sd_buff_din(sd_buff_din),
	.sd_buff_wr(sd_buff_wr),
   .sd_blk_cnt(sd_blk_cnt), // 0 = only 1 block of data at a time

	.img_mounted(img_mounted),
	.img_size(img_size),
	.img_readonly(img_readonly),
	
	.ps2_key(ps2_key),
	.joystick_0(joy0),
	.joystick_1(joy1)

);
// Perform any status flag updates as needed
always @(posedge clk_sys) begin
   status_update = 0;
	
	// If the mounted tape image is readonly, set the OSD flag to true
	if(img_readonly_r) begin
	   status_o = status;
		status_o[2] = 1'b1;
		status_update = 1;
	end
end


//Enable/Disable Hidden Developer Menu via ALT-F11 hotkey
wire       pressed = ps2_key[9];
reg        btn_alt = 0;
reg        dev_menu_en;
reg  [4:0] ram_banks;

assign ram_banks = dev_menu_en ? status[17:13] : 5'd0; //Capture what banks of RAM are enabled, but only if Dev Menu is enabled

always @(posedge clk_sys) begin
	reg old_state;
	old_state <= ps2_key[10];
	
	if(old_state != ps2_key[10]) begin
		casex(ps2_key[7:0])
			'h11: btn_alt   <= pressed; // ALT
			'h78: if(btn_alt) dev_menu_en <= (~dev_menu_en & pressed) | (dev_menu_en & ~pressed);  // F11
		endcase
	end
end


///////////////////////   CLOCKS   ///////////////////////////////

wire clk_sys;
wire clk_100m;
wire clk_25m;
wire clk_10m7;
wire pll_locked;

assign clk_sys = clk_10m7;

pll pll
(
	.refclk(CLK_50M),
	.rst(0),
	.outclk_0(clk_100m),
	.outclk_1(clk_25m),
	.outclk_2(clk_10m7),
	.locked(pll_locked)
);

reg ce_25m = 0;
always @(posedge clk_100m) begin
	reg [2:0] div;
	
	div <= div+1'd1;
	ce_25m <= !div[1:0];
end

wire reset = RESET | status[0] | buttons[1] | ioctl_download | download_reset | ~valid_rom; 

// reset after download
reg [7:0] download_reset_cnt;
wire download_reset = download_reset_cnt != 0;

always @(posedge CLK_50M) begin
	if(ioctl_download || status[0] || buttons[1] || RESET || erasing) download_reset_cnt <= 8'd255;
	else if(download_reset_cnt != 0) download_reset_cnt <= download_reset_cnt - 8'd1;
end

reg valid_rom;

always @(posedge clk_sys) begin
	if(system_rom_hash == 16'h1127 || system_rom_hash == 16'h23D7 || system_rom_hash == 16'h2E8E) valid_rom = 1'b1;
	else valid_rom = 1'b0;
end
//Bring up OSD when no system/boot rom is loaded - copied from Megadrive/Genesis core
reg osd_btn = 0;

always @(posedge clk_sys) begin
	integer timeout = 0;
	reg     has_bootrom = 0;
	reg     last_rst = 0;

	if (RESET) last_rst = 0;
	if (status[0]) last_rst = 1;

	if (ioctl_index == 3 && ioctl_wr & status[0]) has_bootrom <= 1;
   if (~status[2] && valid_rom) has_bootrom <= 1;
	
	if(last_rst & ~status[0]) begin
		osd_btn <= 0;
		if(timeout < 24000000) begin
			timeout <= timeout + 1;
			osd_btn <= ~has_bootrom;
		end
	end
end

///////////////////////// Erase Cart Ram /////////////////////////
reg erasing;
wire [16:0] erase_addr;
wire        erase_wr;

always @(posedge clk_sys) begin
	reg old_clear = 0;
	old_clear <= status[1];
	if (~old_clear & status[1]) begin
		erasing <= 1;
		erase_addr <= 17'h0C000;
		erase_wr <= 1;
	end
	if(erasing == 1) begin
		if(~erase_wr) begin
         if(erase_addr >= 17'h0C000 && erase_addr <= 17'h0DFFF) begin
				erase_wr <= 1;
				erase_addr <= erase_addr + 8'd1;
			end
			else if(erase_addr == 17'hE000) erase_addr <= 17'h10000;
			if(erase_addr >= 17'h10000 && erase_addr <= 17'h18000) begin
				erase_wr <= 1;
				erase_addr <= erase_addr + 8'd1;
			end
			else begin
				erase_addr <= 17'h00000;
				erasing <= 0;
				erase_wr <= 0;
			end
		end
		else erase_wr <= 0;
	end
end

//////////////////////////////////////////////////////////////////

////////////////////////   VIDEO   ///////////////////////////////

assign CLK_VIDEO = clk_100m;

wire ar = status[11];
wire vga_de;
video_freak video_freak
(
	.*,
	.VGA_DE_IN(vga_de),
	.ARX((!ar) ? 12'd400 : ar ),
	.ARY((!ar) ? 12'd300 : 12'd0),
	.CROP_SIZE(status[12] ? 10'd384 : 0),
	.CROP_OFF(0),
	.SCALE(status[10:9])
);

wire [3:0] R,G,B;
wire hblank, vblank;
wire hsync, vsync;


reg hs_o, vs_o;
always @(posedge CLK_VIDEO) begin
	hs_o <= ~hsync;
	if(~hs_o & ~hsync) vs_o <= ~vsync;
end

assign VGA_SL    = 2'd0;

video_mixer #(.LINE_LENGTH(640), .GAMMA(0), .HALF_DEPTH(1)) video_mixer_f18a
(

	.CLK_VIDEO(CLK_VIDEO),
	.ce_pix(ce_25m),
	.CE_PIXEL(CE_PIXEL),

	.R(R),
	.G(G),
	.B(B),


	// Positive pulses.
	.HSync(hs_o),
	.VSync(vs_o),
	.HBlank(hblank),
	
	.freeze_sync(),
	.VGA_R(VGA_R),
	.VGA_G(VGA_G),
	.VGA_B(VGA_B),
	.VGA_VS(VGA_VS),
	.VGA_HS(VGA_HS),
	.VGA_DE(vga_de)

);


////////////////////////   RAM   ///////////////////////////////
wire [15:0] ram_a;
wire        ram_we_n, ram_ce_n;
wire [15:0] ram_di;
wire [15:0] ram_do;
wire  [1:0] ram_be_n;

dpram16_8 #(16) ram
(
	.clock(clk_sys),
	.address_a(ram_a),
	.wren_a(~(ram_we_n | ram_ce_n)),
	.data_a(ram_do),
	.q_a(ram_di),
	.byteena_a(~ram_be_n),

	.wren_b(erasing ? erase_wr : ioctl_wr),
	.address_b(erasing ? {1'b1, erase_addr} : download_addr),
	.data_b(erasing ? 8'd0 : ioctl_dout)
);

wire  [16:0] download_addr;

assign download_addr[0] = ~ioctl_addr[0]; //endian fix
assign download_addr[16:1] = ioctl_index == 2 ? ioctl_addr[16:1] + 16'h8000 : ioctl_addr[16:1];

wire [16:0] cart_size;
wire [16:0] system_rom_hash;
// If the rom being loaded is 32K in size and has bytes x2000-3FFF zero'd out, it's a 24K rom with 16K Ram. 8K @ x6000-7FFF and 8K @ C000-DFFF
wire  [7:0] ram24kcheck;
reg         ram_cart_en;  // Enable the 16K of RAM if criteria is met.

//Tutor = x2E8E
//Pyuta = x23D7
//PyutaJR = x1127

always @(posedge clk_sys) begin
	reg [16:0] last_ioaddr;
	if(ioctl_download && (ioctl_index == 0 || ioctl_index == 1) && ioctl_addr[16:0] == 16'h0) system_rom_hash = 0;
	if(ioctl_download && ioctl_index == 2 && ioctl_addr[16:0] == 16'h0) ram24kcheck = 0;
	if(ioctl_download && ioctl_index == 2) cart_size = ioctl_addr[16:0];
	last_ioaddr <= ioctl_addr[16:0];
	if(ioctl_download == 1 && (ioctl_index == 0 || ioctl_index == 1)  && last_ioaddr != ioctl_addr ) begin
		system_rom_hash = system_rom_hash + ioctl_dout[0];
	end
	if(ioctl_download == 1 && ioctl_index == 2 && last_ioaddr != ioctl_addr && ioctl_addr >= 'h2000 && ioctl_addr <= 'h3FFF ) begin
		ram24kcheck = ram24kcheck + ioctl_dout[0];
	end
end

assign ram_cart_en = (cart_size > 'h4000 && ~ram24kcheck);



//////////////////////////  TAPE  ////////////////////////////////


wire tape_in;
wire tape_adc, tape_adc_act;

assign tape_in = tape_adc_act && tape_adc;

ltc2308_tape #(.CLK_RATE(10738636), .ADC_RATE(41000)) ltc2308_tape
(
  .clk(clk_sys),
  .ADC_BUS(ADC_BUS),
  .dout(tape_adc),
  .active(tape_adc_act)
);

reg       tape_mounted = 0;
reg       tape_changed = 0;

always @(posedge clk_sys) begin

	reg img_mountedD;
	reg [29:0] blocks;
	
	if (tape_changed) begin
		tape_mounted <= 1'b1;
		tape_changed <= 1'b0;
	end
	img_mountedD <= img_mounted;
	if (~img_mountedD && img_mounted) begin
		tape_size <= img_size;
		if(img_size > 0) begin
			if(tape_mounted) begin
				tape_mounted <= 1'b0;
				tape_changed <= 1'b1;
			end
			else tape_mounted <= 1'b1;
			img_readonly_r <= img_readonly;
		end
		else begin
			tape_mounted <= 1'b0;
			img_readonly_r <= 1'b0;
		end
	end
end


///////////////////////   CONSOLE   //////////////////////////////

tutor console
(
	.clk(clk_sys),
	.clk_100m(clk_100m),
	.clk_25m(clk_25m),
	.clk_10m7(clk_10m7),
	.reset(reset),

	.system_type(system_rom_hash == 16'h1127 ? 2'd2 : system_rom_hash == 16'h23D7 ? 2'd1 : system_rom_hash == 16'h2E8E ? 2'd0 : 2'd3),
	.cart_size(cart_size),
	
	.ram({ram_banks[4] | ram_cart_en, ram_banks[3:2],ram_banks[1] | ram_cart_en, ram_banks[0]}),

	////////////// Control Interface //////////////
	.ps2_key(ps2_key),
	.joy0(joy0),
	.joy1(joy1),

	////////////// CPU RAM Interface //////////////
	.cpu_ram_a_o(ram_a),
	.cpu_ram_we_n_o(ram_we_n),
	.cpu_ram_ce_n_o(ram_ce_n),
	.cpu_ram_be_n_o(ram_be_n),
	.cpu_ram_d_i(ram_di),
	.cpu_ram_d_o(ram_do),
	
	///////////// Audio Interface /////////////////
	.audio_o(audio),

	///////////// Tape Interface  /////////////////
	.tape_overlay_en(status[4:3]),
	.tape_ctrl({status[7],status[6]}),  // Currently 2 bits: 1 = Rewind, 0 = Stop
	.tape_in(tape_in),
	.tape_audio_en(status[5]),
	
	.tape_mounted(tape_mounted),
	.tape_readonly(tape_readonly),
	.tape_size(tape_size),
	.sd_lba(sd_lba[0]),
	.sd_rd(sd_rd),
	.sd_wr(sd_wr),
	.sd_ack(sd_ack),
	.sd_strobe(sd_buff_wr),
	.sd_buff_addr(sd_buff_addr),
	.sd_dout(sd_buff_dout),
	.sd_din(sd_buff_din[0]),

	///////////// RGB Video Interface /////////////
	.rgb_r_o(R),
	.rgb_g_o(G),
	.rgb_b_o(B),
	.hsync_n_o(hsync),
	.vsync_n_o(vsync),
	.hblank_o(hblank),
	.sprite_max(status[8])
	
);


endmodule
