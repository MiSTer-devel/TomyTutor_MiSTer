// Tomy Tutor by Flandango
//
// BRAM Layout:
// 0000-FFFF : System Rom and Ram
// 10000-    : Cartridge Rom.  Carts < 16k banked at x8000, > 16k banked at x4000
// All Systems:
// 0000-3FFF Not Banked
// Tutor & Pyuta:
// 4000-7FFF GBasic rom. Banked.  If Cart is over 16k (x4000), then mapped to x0000 of cart memory
// Tutor Only:
// 8000-BFFF Basic Rom.  Banked.  If Cart is enabled and size is 16K and lower,
//                       then mapped to x0000 of cart memory, otherwise if larger, mapped to x4000 of cart memory.

module tutor
(
	input         clk,
	input         clk_100m,
	input         clk_25m,
	input         clk_10m7,
	input         reset,

	input   [1:0] system_type,
	input  [16:0] cart_size,

	////////////// Control Interface //////////////
	input  [10:0] ps2_key,
	input  [31:0] joy0,
	input  [31:0] joy1,
	
	////////////// CPU RAM Interface //////////////
	output [15:0] cpu_ram_a_o,
	output reg    cpu_ram_ce_n_o,
	output reg    cpu_ram_we_n_o,
	output  [1:0] cpu_ram_be_n_o,
	input  [15:0] cpu_ram_d_i,
	output [15:0] cpu_ram_d_o,

	///////////// Audio Interface /////////////////
	output [10:0] audio_o,
	
	///////////// Tape Interface  /////////////////
	input   [1:0] tape_overlay_en,   // 0=Never, 1=Always, 2=Tape Mounted, 3=Tape Activity
	input   [1:0] tape_ctrl,
	input         tape_in,
	input         tape_audio_en,
	
	input         tape_mounted,
	input         tape_readonly,
	input  [31:0] tape_size,
	output [31:0] sd_lba,
	output        sd_rd,
	output        sd_wr,
	input         sd_ack,
	input         sd_strobe,
	input   [9:0] sd_buff_addr,
	input   [7:0] sd_dout,
	output  [7:0] sd_din,
	output reg    tape_out, 

	///////////// RGB Video Interface /////////////
	output  [3:0] rgb_r_o,
	output  [3:0] rgb_g_o,
	output  [3:0] rgb_b_o,

	output reg    hsync_n_o,
	output reg    vsync_n_o,
	output reg    hblank_o,
	
	output  [9:0] raster_x,
	output  [9:0] raster_y,
	input  reg    sprite_max
);


//  -----------------------------------------------------------------------------
//  -- TMS9928A Video Display Processor
//  -----------------------------------------------------------------------------
// VDP read and write signals
wire        vdp_wr;
wire        vdp_rd;
wire        vdp_interrupt; //low true
wire  [7:0] vdp_data_out;
reg 	[3:0] R,G,B;   // RGB from F18A

f18a_core f18a
(
	.clk_100m0_i(clk_100m),
	.clk_25m0_i(clk_25m),
	.reset_n_i(~reset),
	.mode_i(cpu_addr[1]),
	.csr_n_i(~vdp_rd),
	.csw_n_i(~vdp_wr),
	.int_n_o(vdp_interrupt),
	.cd_i(data_from_cpu[15:8]),
	.cd_o(vdp_data_out),
	
//      -- Video Output
	.blank_o(hblank_o),
	.hsync_o(hsync_n_o),
	.vsync_o(vsync_n_o),
	.red_o(R),
	.grn_o(G),
	.blu_o(B),

//      -- Feature Selection
	.sprite_max_i(sprite_max), //0 = 32, 1 = 4
	.scanlines_i(1'b0),        //0 = Off, 1 = On
	
//	     -- Extras - Added to accomodate overlay
	.raster_x(raster_x),
	.raster_y(raster_y),
	
//      -- SPI to GPU
	.spi_clk_o(),
	.spi_cs_o(),
	.spi_mosi_o(),
	.spi_miso_i(1'b0)
);


ls74 flipflop
(
	.q1(cpu_ready),
	.n_clr1(1'b1),
	.n_pre1(1'b1),
	.d1(psg_ready_s),
	.clk1(clkout)
);


// TMS9995 utilities (flags, timer, etc.). Provides the functions
// not present in other 99xx CPU's.
//
reg  utl_sel, utl_cruin;
wire [15:0] utl_o;
wire [3:0] cpu_ic;

UTIL9995 utl(
	.clk(clkout),
	.rst(reset),
 
	.ab(cpu_addr),
	.din(data_from_cpu),
	.dout(utl_o),
	.nwr(~cpu_wr),
	.nmemen(nMEM),
	.utl_sel(utl_sel),
 
	.cruclk(~cpu_cruclk),
//	.cruclk(cpu_cruclk),
	.cruout(cpu_cruout),
   .cruin(utl_cruin),
 
	.int1(1'b0),
	.int4(tape_int_en ? tape_int : 1'b0),
	.irq(cpu_int_req),
	.ic(cpu_ic),
	.bst(cpu_bst)
);


TMS99095 cpu
(
//	.CLK(clk_2m7),
	.CLK(clk),
	.CLKOUT(clkout),
//	.CLKOUT(clk_2m7),
	.RESET(cpu_reset),
	.ADDR_OUT(cpu_addr),
	.ADDR_OUT2(),
	.DATA_IN(data_to_cpu),
	.DATA_OUT(data_from_cpu),
	.RD(cpu_rd),
	.WR(cpu_wr),
	.nMEM(nMEM),
	.READY(cpu_ready),
	.IAQS(cpu_iaq),
	.AS(cpu_as),
	.INT(cpu_int_req),
	.IC(cpu_ic),
	.NMI(1'b0),
	.CRUIN(cpu_cruin),
	.CRUOUT(cpu_cruout),
	.CRUCLK(cpu_cruclk),
	.HOLD(cpu_hold),
	.BST(cpu_bst)
);

wire [15:0] cpu_addr;
wire [15:0] data_to_cpu;	// data to CPU
wire [15:0] data_from_cpu;	// data from CPU
wire  [3:0] cpu_bst;
wire        clkout;			// 2.7Mhz clock from cpu (1/4 of input clock)
wire        cart_en;
wire        nMEM;
wire        cpu_reset;
wire        cpu_rd;
wire        cpu_wr;
wire        cpu_ready;
wire        cpu_iaq;
wire        cpu_as;
wire        cpu_cruin;
wire        cpu_cruout;
wire        cpu_cruclk;
	
wire        cpu_hold;
	
wire        cpu_int_req;
wire        cpu_int_ack;


// Scratchpad
//wire [15:0] wp_ram;
//spram #(11, 16) wpram
//(
//	.clock(clk),
//	.address(cpu_addr[11:1]),
//	.wren((cpu_wr == 1'b1 && cpu_addr[15:12] == 4'b1111)),
//	.data(data_from_cpu),
//	.q(wp_ram)
//);


// Data Bus
wire [15:0] sram_addr_bus; 
wire [15:0] sram_16bit_read_bus;

assign cpu_ram_a_o = sram_addr_bus;
assign cpu_ram_be_n_o = 2'b00;
assign cpu_ram_ce_n_o = cpu_bst[3];
assign sram_16bit_read_bus = cpu_ram_d_i;
assign cpu_reset = reset;
assign cpu_hold = 'b0;

assign data_to_cpu = vdp_rd ? {vdp_data_out,8'h00} : 
							utl_sel ? utl_o :
							(system_type == 2'd2 && cpu_addr[15:11] == 5'h1D) ? {kbdjr_scan,8'h00} :
							cpu_addr[15:8] == 8'hE1 ? 16'h0000 : 
//							cpu_addr[15:12] == 4'b1111 ? wp_ram : 
							cpu_addr == 'hE110 ? 16'h0000 : 						//Return 'h4200 if we have alternate boot rom in cart, 0 if not.
							sram_16bit_read_bus[15:0];

assign sram_addr_bus =
	((cart_en || system_type != 2'd0) && cart_size > 16'h4000 && (cpu_addr >= 16'h4000 && cpu_addr <= 16'hDFFF)) ? cpu_addr[15:1] + 16'h6000 : 
	((cart_en || system_type != 2'd0) && cart_size <= 16'h4000 && (cpu_addr >= 16'h8000 && cpu_addr <= 16'hDFFF)) ? cpu_addr[15:1] + 16'h4000 : 
	{1'b0,cpu_addr[15:1]};


reg last_wr;
always @(posedge clk) 
begin 
  last_wr <= cpu_wr;
end

assign audio_o = {1'b0 ,aout_o[7],tape_audio_en ? tape_reading ? tape_in | tape_file_in : tape_writing ? tape_out : aout_o[6] : aout_o[6], aout_o[5:0], 2'b00};
assign cpu_ram_we_n_o = (cpu_wr == 1'b1 && cpu_addr[15:12] == 4'b1111) ? 1'b0 : 1'b1;

assign cpu_ram_d_o = data_from_cpu;

assign vdp_rd = (cpu_addr[15:2] == 14'h3800 && cpu_rd && cpu_bst == 4'b0001) ? 1'b1: 1'b0;
assign vdp_wr = (cpu_addr[15:2] == 14'h3800 && cpu_wr && !last_wr) ? 1'b1: 1'b0;

wire [3:0] wr_sampler;
wire [3:0] rd_sampler;

always @(posedge clk) begin
	wr_sampler = {wr_sampler[2:0],cpu_wr};
	rd_sampler = {rd_sampler[2:0],(cpu_rd && cpu_bst == 4'b0001)};
end

// Cartridge Handling
always @(posedge clk) begin
	if(reset) begin
		cart_en = 1'b0;
	end
	if (cpu_addr == 'hE108 && cpu_wr) cart_en = 1'b0;
	if (cpu_addr == 'hE10C && cpu_wr) cart_en = 1'b1;
end



////////////////////////  OVERLAY  ///////////////////////////////

reg [9:0] tape_max_blocks = 0;
reg [9:0] tmb_1, tmb_2, tmb_3;	//Tape Maximum Blocks
reg [9:0] tcb_1, tcb_2, tcb_3;	//Tape Current Block
reg [3:0] tape_info;
reg [3:0] tape_activity;

assign tmb_1 = tape_max_blocks[9:0] / 10'd100;
assign tmb_2 = (tape_max_blocks[9:0] % 10'd100) / 10'd10;
assign tmb_3 = (tape_max_blocks[9:0] % 10'd100) % 10'd10;
assign tcb_1 = (sd_lba[9:0] / 10'd100);
assign tcb_2 = (sd_lba[9:0] % 10'd100) / 10'd10;
assign tcb_3 = (sd_lba[9:0] % 10'd100) % 10'd10;
assign tape_info = {3'd0,tape_readonly};

reg  [7:0] ro,go,bo;
always @(posedge clk) begin

	reg [29:0] blocks;
	
	if (tape_max_blocks >999) tape_max_blocks <= 999;
	if (tape_mounted) begin
	   blocks = tape_size[29:0] / 30'd1024;
		tape_max_blocks <= blocks[9:0];
		if(tape_size[29:0] % 30'd1024) tape_max_blocks <= tape_max_blocks + 1'b1;  //If the tape size is not on 1k boundaries, round up
	end
end

tape_status_overlay #(.X(2), .Y(15)) tapeOverlay
(
	.clk(clk_100m),
	.en(tape_overlay_en == 1 ? 1'b1 : tape_overlay_en == 2 ? tape_mounted : tape_overlay_en == 3 ? (tape_reading | tape_writing) : 1'b0),
	.data({tape_info, tape_activity == 4'd0 ? 4'd11 : tape_activity, tcb_1[3:0], tcb_2[3:0], tcb_3[3:0], tmb_1[3:0], tmb_2[3:0], tmb_3[3:0]}),
	.num_pix({1'b0,raster_x}),
	.num_line({1'b0,raster_y}),
	.r_i({R,R}),
	.g_i({G,G}),
	.b_i({B,B}),
	.r_o(ro),
	.g_o(go),
	.b_o(bo)
);

assign rgb_r_o = ro[3:0];
assign rgb_g_o = go[3:0];
assign rgb_b_o = bo[3:0];

// Cassette Handling
wire  [9:0] tape_mem_addr;
wire  [7:0] tape_in_acc;
wire  [7:0] tape_out_acc;
wire  [3:0] tape_bit_cnt;
wire  [7:0] tape_data_in;
reg   [7:0] tape_wr_sampler;

wire  [5:0] tape_in_pulse;
reg   [2:0] tape_pulse_ptr;
wire tape_in_bit;

wire tape_en;
wire tape_file_in;
wire tape_int;
wire tape_int_en;
wire tape_mem_rd;
wire tape_stop;
wire tape_eot;

wire tape_44k1_en;
wire [12:0] tape_44k1_div;


reg  advance_sd_lba, sd_done_writing;

reg  tape_writing, tape_reading, tape_recording_stopped;
assign tape_activity = tape_mounted ? (tape_reading ? 4'd1 : tape_writing ? 4'd10 : tape_stop ? 4'd3 : tape_eot ? 4'd9 : 4'd8) : 4'd11;
always @(posedge clk) begin
	reg old_sd_done_writing, old_tape_mounted, old_tape_mem_rd;
	if(reset) begin
		tape_en <= 1'b0;
		tape_mem_addr <= 10'h000;
		tape_out_acc <= 'h00;
		tape_bit_cnt <= 4'h0;
		tape_in_acc <= 8'd0;
		tape_stop <= 1'b0;
		tape_int_en <= 1'b0;
		tape_44k1_div <= 13'h0000;
		tape_in_pulse <= 6'b000000;
		tape_pulse_ptr <= 3'b111;
		sd_lba <= 0;
		sd_card_write <= 0;
		tape_writing <= 0;
		if(tape_size > 0) sd_card_read <= 1;
		else sd_card_read <= 0;
		tape_wr_sampler <= 8'h00;
		tape_recording_stopped <= 1'b0;
		tape_out <= 0;
		tape_eot <= 0;
	end
	else begin
		old_sd_done_writing <= sd_done_writing;
		old_tape_mounted <= tape_mounted;
		old_tape_mem_rd <= tape_mem_rd;
		
		
		if(old_tape_mem_rd && ~tape_mem_rd) tape_in_acc <= tape_data_in;  //Wait one cycle after a tape_mem_rd to load accumulator
		sd_card_read <= 0;
		sd_card_write <= 0;
		
		if(advance_sd_lba || (~old_sd_done_writing && sd_done_writing && tape_mem_addr == 10'h000)) begin
			sd_lba <= sd_lba + 1'b1;
			sd_card_read <= 1'b1;
			if(advance_sd_lba) advance_sd_lba <= 1'b0;
		end
		else if (~old_sd_done_writing && sd_done_writing && tape_mem_addr != 10'h000) begin //SD Write triggered by end of save/write so advance tape_mem_addr by one
			tape_mem_addr <= tape_mem_addr + 1'b1;
		end
		
		if((sd_lba * 1024 + tape_mem_addr) > tape_size) begin 			//Reached end of tape so stopping
			tape_stop <= 1'b1;
			tape_reading <= 1'b0;
			tape_writing <= 1'b0;
			tape_en <= 1'b0;
			tape_eot <= 1'b1;
		end

	//Handle Tape Writing	
		if(~tape_readonly && tape_mounted) begin				//Don't bother with any of the tape writing routines if there is no tape or it's write protected
			if(tape_mem_wr == 1'b1 && ~tape_stop) begin
				tape_mem_wr <= 1'b0;
				tape_out_acc <= 8'h0;
				tape_bit_cnt <= 4'h0;
				
				if(tape_recording_stopped) begin             //We are done recording this session so write current fifo to disk
					sd_card_write <= 1;
					tape_recording_stopped <= 1'b0;
					tape_writing <= 1'b0;
					tape_bit_cnt <= 4'h0;
					tape_stop <= 1'b1;
				end
				
				if(tape_mem_addr == 10'h3FF) begin				//Fifo Mem Filled, write to SD card, reset address to 0
					sd_card_write <= 1;
					tape_mem_addr <= 10'h000;
				end
				else tape_mem_addr <= tape_mem_addr + 1'b1;
			end

			if (cpu_addr == 'hEE00 && cpu_wr && wr_sampler[1:0] == 2'b01 && tape_en) begin
				tape_wr_sampler <= {tape_wr_sampler[6:0],1'b0};
				tape_out <= 1'b0;
			end
			if (cpu_addr == 'hEE20 && cpu_wr && wr_sampler[1:0] == 2'b01 && tape_en) begin
				tape_wr_sampler <= {tape_wr_sampler[6:0],1'b1};
				tape_out <= 1'b1;
			end

			if (tape_wr_sampler[4:0] == 5'b00100) begin
				tape_out_acc <= {tape_out_acc[6:0],1'b0};
				if(tape_en) tape_writing <= 1;
				if (tape_bit_cnt == 4'h7) begin
					tape_mem_wr <= 1'b1;
				end
				else tape_bit_cnt <= tape_bit_cnt + 1'b1;
				tape_wr_sampler <= 8'h00;
			end
			else if (tape_wr_sampler[4:0] == 7'b0010100) begin
				tape_out_acc <= {tape_out_acc[6:0],1'b1};
				if(tape_en) tape_writing <= 1;
				if (tape_bit_cnt == 4'h7) begin
					tape_mem_wr <= 1'b1;
				end
				else tape_bit_cnt <= tape_bit_cnt + 1'b1;
				tape_wr_sampler <= 8'h00;
			end

		end

	//Handle Tape Reading
	
		//Mimic 44100HZ timing for reading tape files
		if(tape_mounted) begin
			tape_mem_rd <= 0;
			advance_sd_lba <= 0;
			if(tape_mem_rd && ~tape_stop) begin
				tape_bit_cnt <= 4'h0;
				if(tape_mem_addr == 10'h3FF) begin
					advance_sd_lba <= 1'b1;
				end
			end

			if(tape_reading && tape_mounted) begin
				if(tape_44k1_div == 13'h157C) begin
					tape_44k1_en = 1'b1;
					tape_44k1_div <= 'h0;
				end
				else begin
					tape_44k1_en = 1'b0;
					tape_44k1_div <= tape_44k1_div + 1'b1;
				end
			end

			if (tape_44k1_en) begin
				tape_file_in = tape_in_pulse[tape_pulse_ptr];
				tape_pulse_ptr <= tape_pulse_ptr - 1'b1;
			end
			
			if(tape_int_en) tape_int = tape_in | tape_file_in;

			if(tape_pulse_ptr == 3'b111 && tape_reading) begin
				tape_in_bit = tape_in_acc[7];
				tape_in_acc <= {tape_in_acc[6:0],1'b0};
				
				if (tape_bit_cnt == 4'h7 && ~tape_stop) begin
					tape_mem_addr <= tape_mem_addr + 1'b1;
					tape_mem_rd <= 1'b1;
				end
				else tape_bit_cnt <= tape_bit_cnt + 1'b1;
				if(tape_in_bit) begin
					tape_in_pulse <= 6'b010100;
					tape_pulse_ptr <= 3'd4;
				end
				else begin
					tape_in_pulse <= 6'b011000;
					tape_pulse_ptr <= 3'd5;
				end
			end
		end
		
		if (cpu_addr == 'hEE40 && cpu_wr && wr_sampler[1:0] == 2'b01) begin
			if(~tape_int_en) begin
				tape_int_en <= 1'b1;
			end
			if(tape_en) tape_reading <= 1'b1;
		end
				
		else if (cpu_addr == 'hEE60 && cpu_wr && wr_sampler[1:0] == 2'b01) begin
			tape_int = 1'b0;
			tape_int_en <= 1'b0;
			if(tape_en) tape_reading <= 1'b1;
		end
		
		//Tape Motor Control
		else if (cpu_addr == 'hEEC0 && cpu_wr && wr_sampler[1:0] == 2'b01) begin		//Start Tape Motor
			tape_en <= 1'b1;
			if(~tape_eot) tape_stop <= 1'b0;
		end
		else if (cpu_addr == 'hEEE0 && cpu_wr && wr_sampler[1:0] == 2'b01) begin		//Stop Tape Motor
			tape_en <= 1'b0;
			if(tape_reading) begin
				tape_stop <= 1'b1;
				tape_reading <= 1'b0;
			end
			else if(tape_writing) begin
				tape_mem_wr <= 1'b1;       //Write the last byte to fifo
				tape_recording_stopped <= 1'b1;
			end
		end
		
		
		
	//Tape Controls
		if (tape_ctrl[0] || btn_stop) begin									//Tape Stop Pressed, set Stop Flag
			tape_stop <= 1'b1;
			tape_reading <= 1'b0;
			tape_writing <= 1'b0;
			tape_en <= 1'b0;
			tape_wr_sampler <= 8'h00;
		end
		if (tape_ctrl[1] || btn_rewind || (old_tape_mounted != tape_mounted)) begin									//Tape Rewind Pressed or Tape (Un)Mounted
			tape_stop <= 1'b0;									//  Clear Stop Flag
			tape_bit_cnt <= 4'h0;
			tape_out_acc <= 8'h0;
			tape_mem_addr <= 10'h0000;
			tape_in_acc <= 8'd0;
			tape_44k1_div <= 13'h0000;
			tape_in_pulse <= 6'b000000;
			tape_pulse_ptr <= 3'b111;
			tape_wr_sampler <= 8'h00;
			sd_lba <= 0;
			if(tape_mounted) sd_card_read <= 1;
			tape_writing <= 0;
			tape_eot <= 0;

		end
	end
end


// ==================================== FIFO ==================================

reg  [10:0] fifo_cpuptr;
reg  [9:0] fifo_cpuptr_adj;
wire [7:0] fifo_q;
reg        s_odd; //odd sector
reg  [9:0] fifo_sdptr;

wire tape_mem_wr;

dpram #(8, 10) fifo
(
	.clock(clk),

	.address_a(sd_buff_addr),
	.data_a(sd_dout),
	.wren_a(sd_strobe & sd_ack),
	.q_a(sd_din),

	.address_b(tape_mem_addr),
	.data_b(tape_out_acc),
	.wren_b(tape_mem_wr),
	.q_b(tape_data_in)
);

// ------------------ SD card control ------------------------
localparam SD_IDLE = 0;
localparam SD_READ = 1;
localparam SD_WRITE = 2;

reg [1:0] sd_state;
reg [31:0] old_sd_lba;
reg [9:0] old_tape_mem_addr;
reg       sd_card_write;
reg       sd_card_read;

always @(posedge clk) begin
	reg sd_ackD;
	reg sd_card_readD;
	reg sd_card_writeD;

	sd_card_readD <= sd_card_read;
	sd_card_writeD <= sd_card_write;
	sd_ackD <= sd_ack;
	
	if (sd_ack) {sd_rd, sd_wr} <= 0;
	if (sd_done_writing && (old_sd_lba != sd_lba || old_tape_mem_addr != tape_mem_addr)) sd_done_writing <= 0;

	case (sd_state)
	SD_IDLE:
	begin
		if (~sd_card_readD & sd_card_read) begin
			sd_rd <= 1;
			sd_state <= SD_READ;
		end
		else if (~sd_card_writeD & sd_card_write) begin
			sd_wr <= 1;
			sd_state <= SD_WRITE;
		end
	end

	SD_READ:
	if (sd_ackD & ~sd_ack) begin
		sd_state <= SD_IDLE;
	end

	SD_WRITE:
	if (sd_ackD & ~sd_ack) begin
		sd_state <= SD_IDLE;
		old_sd_lba <= sd_lba;
		old_tape_mem_addr <= tape_mem_addr;
		sd_done_writing <= 1;
	end

	default: ;
	endcase
end

////CRU
wire flagsel = (cpu_addr[15:5] == 11'h0F7); // CRU 1EE0-1EFE
wire kbdsel = (cpu_addr[15:7] == 9'h1D8);   // KEYBOARD CRU EC00-EC7F
wire tapesel = (cpu_addr == 16'hED00);      // CASSETTE CRU ED00

// CRUIN multiplexer
//
assign cpu_cruin = flagsel ? utl_cruin : kbdsel ? keyboard[cpu_addr[6:1]] : tapesel ? tape_in | tape_file_in : 1'b1;


always @(posedge clk) begin
	if(reset) begin
		audio_we <= 1'b0;
		audio_data_out <= 8'd0;
	end
	else begin
		if (cpu_wr) begin
			if (cpu_addr == 'hE200) begin
				audio_we <= 1'b1;		// Audio chip write
				audio_data_out <= data_from_cpu[15:8];
			end
		end
		if (psg_ready_s && audio_we) audio_we <= 1'b0;
	end
end


//Sound Clock 3.58mhz
wire clk_en_3m58_s;
 
always @(posedge clk_10m7) begin
	reg [2:0] div;
	if(reset) begin
		div <= 0;
		clk_en_3m58_s <= 0;
	end
	else begin
		div <= div+1'd1;
		if (div == 2) begin
			clk_en_3m58_s <= 1;
			div <= 0;
		end
		else clk_en_3m58_s <= 0;
	end
end
//Sound
wire audio_we;
wire psg_ready_s;
wire [7:0] audio_data_out;
wire [7:0] aout_o;
sn76489_top #(.clock_div_16_g(1)) audio
(
      .clock_i(clk),
      .clock_en_i(clk_en_3m58_s),
      .res_n_i(~reset),
      .ce_n_i(~audio_we),
      .we_n_i(~audio_we),
      .ready_o(psg_ready_s),
      .d_i(audio_data_out),
      .aout_o(aout_o)
);


//Keyboard
wire       pressed = ps2_key[9];
wire [8:0] code    = ps2_key[8:0];

//Handle Keyboard for Pyuta Jr
wire [7:0] kbdjr_scan;
assign kbdjr_scan = (cpu_addr == 16'hE800) ? keyboardJr[24:31] : 
                    (cpu_addr == 16'hEA00) ? keyboardJr[16:23] :
						  (cpu_addr == 16'hEC00) ? keyboardJr[8:15]  :
						  (cpu_addr == 16'hEE00) ? keyboardJr[0:7]   :
						  8'h0000;

always @(posedge clk) begin
	reg old_state;
	old_state <= ps2_key[10];
	
	if(old_state != ps2_key[10]) begin
		casex(code[7:0])
			'h75: btn_up    <= pressed;
			'h72: btn_down  <= pressed;
			'h6B: btn_left  <= pressed;
			'h74: btn_right <= pressed;
			
			'h16: btn_1     <= pressed; // 1
			'h1E: btn_2     <= pressed; // 2
			'h26: btn_3     <= pressed; // 3
			'h25: btn_4     <= pressed; // 4
			'h2E: btn_5     <= pressed; // 5
			'h36: btn_6     <= pressed; // 6
			'h3D: btn_7     <= pressed; // 7
			'h3E: btn_8     <= pressed; // 8
			'h46: btn_9     <= pressed; // 9
			'h45: btn_0     <= pressed; // 0
			'h4E: btn_min   <= pressed; // -
			'h55: btn_deg   <= pressed; // = => º

			'h15: btn_q     <= pressed; // q
			'h1D: btn_w     <= pressed; // w
			'h24: btn_e     <= pressed; // e
			'h2D: btn_r     <= pressed; // r
			'h2C: btn_t     <= pressed; // t
			'h35: btn_y     <= pressed; // y
			'h3C: btn_u     <= pressed; // u
			'h43: btn_i     <= pressed; // i
			'h44: btn_o     <= pressed; // o
			'h4D: btn_p     <= pressed; // p
			'h54: btn_us    <= pressed; // [ => _
			'h5D: btn_cb    <= pressed; // \ => ]
			'h5B: btn_ob    <= pressed; // ] => [
			
			'h1C: btn_a     <= pressed; // a
			'h1B: btn_s     <= pressed; // s
			'h23: btn_d     <= pressed; // d
			'h2B: btn_f     <= pressed; // f
			'h34: btn_g     <= pressed; // g
			'h33: btn_h     <= pressed; // h
			'h3B: btn_j     <= pressed; // j
			'h42: btn_k     <= pressed; // k
			'h4B: btn_l     <= pressed; // l
			'h4C: btn_se    <= pressed; // ;
			'h5A: btn_rt    <= pressed; // enter
			
			'h12: btn_sh    <= pressed; // lshift
			'h1A: btn_z     <= pressed; // z
			'h22: btn_x     <= pressed; // x
			'h21: btn_c     <= pressed; // c
			'h2A: btn_v     <= pressed; // v
			'h32: btn_b     <= pressed; // b
			'h31: btn_n     <= pressed; // n
			'h3A: btn_m     <= pressed; // m
			'h41: btn_co    <= pressed; // ,
			'h49: btn_pe    <= pressed; // .
			'h4A: btn_fs    <= pressed; // /
			'h59: btn_sh    <= pressed; // rshift

			'h58: btn_al    <= pressed; // caps => alpha lock
//			'h14: btn_ct    <= pressed; // lctrl
			'h29: btn_sp    <= pressed; // space
			'h66: btn_left  <= pressed; // BackSpace
			'h52: btn_col   <= pressed; // Quotes => ;/*
			'h05: btn_mon   <= pressed; // F1 => MON Key
			'h06: btn_mod   <= pressed; // F2 => MOD Key
			
			'h03: btn_stop  <= pressed; // F5 => Tape Stop
			'h0B: btn_rewind<= pressed; // F6 => Tape Rewind
			
		endcase
	end
end

// Tape Function Buttons:
reg btn_stop = 0;
reg btn_rewind = 0;

// Keyboard
reg btn_1 = 0;
reg btn_2 = 0;
reg btn_3 = 0;
reg btn_4 = 0;
reg btn_5 = 0;
reg btn_6 = 0;
reg btn_7 = 0;
reg btn_8 = 0;
reg btn_9 = 0;
reg btn_0 = 0;
reg btn_min = 0;
reg btn_deg = 0;
reg btn_mon = 0;

reg btn_q = 0;
reg btn_w = 0;
reg btn_e = 0;
reg btn_r = 0;
reg btn_t = 0;
reg btn_y = 0;
reg btn_u = 0;
reg btn_i = 0;
reg btn_o = 0;
reg btn_p = 0;
reg btn_us = 0; //Underscore / @
reg btn_rt = 0;
			
reg btn_a = 0;
reg btn_s = 0;
reg btn_d = 0;
reg btn_f = 0;
reg btn_g = 0;
reg btn_h = 0;
reg btn_j = 0;
reg btn_k = 0;
reg btn_l = 0;
reg btn_se = 0;
reg btn_col = 0;
reg btn_ob = 0; //Open Bracket
			
reg btn_z = 0;
reg btn_x = 0;
reg btn_c = 0;
reg btn_v = 0;
reg btn_b = 0;
reg btn_n = 0;
reg btn_m = 0;
reg btn_co = 0;
reg btn_pe = 0;
reg btn_fs = 0;
reg btn_cb = 0; //Close Bracket

reg btn_al = 0;
reg btn_sh = 0;
reg btn_sp = 0;
reg btn_mod = 0;

reg btn_up    = 0;
reg btn_down  = 0;
reg btn_left  = 0;
reg btn_right = 0;


wire [0:7] keys0 = {btn_1 | joy0[6],    btn_2 | joy0[7],   btn_q,    btn_w,     btn_a,   btn_s,   btn_z,   btn_x };
wire [0:7] keys1 = {btn_3,    btn_4,   btn_e,    btn_r,     btn_d,   btn_f,   btn_c,   btn_v };
wire [0:7] keys2 = {btn_5,    btn_6,   btn_t,    btn_y,     btn_g,   btn_h,   btn_b,   btn_n };
wire [0:7] keys3 = {btn_7,    btn_8,   btn_9,    btn_u,     btn_i,   btn_j,   btn_k,   btn_m };
wire [0:7] keys4 = {btn_0,    btn_min, btn_o | joy0[4],    btn_p | joy0[5],     btn_l | joy0[2],   btn_se | joy0[1],  btn_co | joy0[3],  btn_pe | joy0[0]};
wire [0:7] keys5 = {1'b0,     1'b0,    btn_deg | joy1[4],  btn_us | joy1[5],    btn_col | joy1[2], btn_ob | joy1[1],  btn_fs | joy1[3],  btn_cb | joy1[0]};
wire [0:7] keys6 = {1'b0,     btn_al,  btn_sh,   btn_mon,   btn_rt | joy0[8],  1'b0,    btn_mod, btn_sp};
wire [0:7] keys7 = {btn_left | joy0[1], btn_up | joy0[3],  btn_down | joy0[2], btn_right | joy0[0], 1'b0,    1'b0,    1'b0,    1'b0  };
wire [0:63] keyboard = {keys0, keys1, keys2, keys3, keys4, keys5, keys6, keys7};

wire [0:31] keyboardJr = {joy1[0], joy1[3], joy1[1], joy1[2], joy1[5], joy1[4], 1'b0, 1'b0,
                          btn_right | joy0[0], btn_down | joy0[2], btn_up | joy0[3], btn_left | joy0[1], btn_pe, btn_co, 1'b0, 1'b0,
								  btn_2 | joy0[7], btn_1 | joy0[6], btn_mon | btn_q, btn_mod | btn_m, btn_p, btn_rt | joy0[8], 1'b0, 1'b0,
                          joy0[0], joy0[3], joy0[1], joy0[2], joy0[5], joy0[4], 1'b0, 1'b0};

endmodule


module dpram #(parameter DATAWIDTH=8, ADDRWIDTH=9)
(
	input                   clock,

	input   [ADDRWIDTH-1:0] address_a,
	input   [DATAWIDTH-1:0] data_a,
	input                   wren_a,
	output reg [DATAWIDTH-1:0] q_a,

	input   [ADDRWIDTH-1:0] address_b,
	input   [DATAWIDTH-1:0] data_b,
	input                   wren_b,
	output reg [DATAWIDTH-1:0] q_b
);

reg [DATAWIDTH-1:0] ram[0:(1<<ADDRWIDTH)-1];

always @(posedge clock) begin
	if(wren_a) begin
		ram[address_a] <= data_a;
		q_a <= data_a;
	end else begin
		q_a <= ram[address_a];
	end
end

always @(posedge clock) begin
	if(wren_b) begin
		ram[address_b] <= data_b;
		q_b <= data_b;
	end else begin
		q_b <= ram[address_b];
	end
end

endmodule
