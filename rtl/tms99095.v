//
// TMS99xxx CPU Core
//
// This source code is public domain
//
// The datasheet for the TMS99xxx series is available at:
//   https://ftp.whtech.com/datasheets%20and%20manuals/Datasheets%20-%20TI/TMS99000/MP009_99105A_99110A_Nov82.pdf
//
// Also, the (TTL) schematics of the 990/10 can be found here:
//   http://www.bitsavers.org/pdf/ti/990/schematics/
//
// Based on original code by Paul Ruiz
// Modified by Flandango to work with Mister FPGA's Tomy Tutor Core
//
// Noteworthy changes:
//          - Fixed X instruction to not allow interrupt upon completion to allow read-in instruction to execute.
//          - Added 8 cycles to SWPB to bring it closer to actual hardware.  (Fixes timing issues with Tomy Tutor and games running too fast)


module TMS99095 (
  input  wire         CLK,
  input  wire         RESET,
  input  wire         READY,
  input  wire         INT,
  input  wire         NMI,
  input  wire         CRUIN,
  input  wire         HOLD,
  input  wire  [15:0] DATA_IN,
  input  wire   [3:0] IC,
  output wire  [15:0] ADDR_OUT,
  output wire  [15:0] DATA_OUT,
  output reg          CLKOUT,
  output wire         RD,
  output wire         WR,
  output wire         nMEM,
  output wire         IAQS,
  output wire         AS,
  output wire         CRUOUT,
  output wire         CRUCLK,
  output wire   [3:0] BST
  );
  

  // ==========================================================================
  // Clock generation
  // ==========================================================================
	reg [2:0] clk_div;

	always @(posedge CLK) begin
		if(RESET) begin
			CLKOUT <= 1'b0;
			clk_div <= 3'd1;
		end
		if(clk_div <= 3'd0) begin
			CLKOUT <= ~CLKOUT;
			clk_div <= 3'd1;
		end
		else clk_div <= clk_div - 1'b1;
	end


  // ==========================================================================
  // Cycle correct bus interface
  // ==========================================================================

  wire   [15:0] db_in;

  assign ADDR_OUT = p_bus;
  // Databus byte swapper
  assign DATA_OUT = (byte_bus & p_bus[0]) ? { e_bus[7:0],   e_bus[15:8] }   : e_bus;
  assign db_in    = (byte_bus & p_bus[0]) ? { DATA_IN[7:0], DATA_IN[15:8] } : DATA_IN;

  // auto-insert wait state for cru cycles

  wire cruwr = (ctrl==ldcr) || (ctrl==sboz);
  
  assign RD     = (!nmemen && !holding) && (ebus==e_nn);
  assign WR     = (!nmemen && !holding) && (ebus!=e_nn);
  assign nMEM   = nmemen;
  assign CRUCLK = cruwr & dowait;
  assign CRUOUT = (ctrl==sboz) ? ir[8] : (byte_cru ? t1[8] : t1[0]);
  assign BST    = bst;
  assign IAQS   = (bst==iaq) && !holding;
  assign AS     = 1;
 
  reg stall, dowait, holding;
  
  always @(posedge CLKOUT) holding <= (HOLD & !dowait);
  
  always @(negedge CLKOUT)
  begin
    dowait = (((RD && ~(&p_bus[15:12])) | cruwr) && !(RESET|holding)) ? !dowait : 1'b0;
    stall = dowait | holding | !READY ;
  end

  // ==========================================================================
  // Microcode symbolic signal names
  // ==========================================================================
  
  localparam  sopl  = 4'b0000, sop   = 4'b0001,  iop  = 4'b0010, iaq   = 4'b0011,
              dop   = 4'b0100, inta  = 4'b0101,  ws   = 4'b0110, gm    = 4'b0111,
              aumsl = 4'b1000, aums  = 4'b1001,  rset = 4'b1010, io    = 4'b1011,
              wps   = 4'b1100, sts   = 4'b1101,  mid  = 4'b1110, holda = 4'b1111;
    
  localparam  add   = 4'b0000, sub   = 4'b0001,  ps_a = 4'b0010, ps_b  = 4'b0011,
              abs   = 4'b0100, lxor  = 4'b0101,  lor  = 4'b0110, land  = 4'b0111,
              landn = 4'b1000, cmp   = 4'b1001,  swap = 4'b1010, neg   = 4'b1011,
              nnnn  = 4'b0010;
      
  localparam  a_nn  = 3'b000,
              a_wp  = 3'b000,  a_ea  = 3'b001,  a_pc  = 3'b010,  a_t1  = 3'b011,
              a_t2  = 3'b100,  a_t3  = 3'b101,  a_st  = 3'b110,  a_c0  = 3'b111;  
  
  localparam  b_nn  = 4'b0000,
              b_srg = 4'b0000, b_drg = 4'b0001, b_cb  = 4'b0010, b_t1  = 4'b0011,
              b_t2  = 4'b0100, b_t3  = 4'b0101, b_ofs = 4'b0110, b_cc  = 4'b0111,
              b_sc  = 4'b1000, b_xop = 4'b1001, b_con = 4'b1010, b_cr  = 4'b1011,
              b_int = 4'b1100;
    
  localparam  ___   = 3'b000,
              c_0   = 3'b000,  c_1   = 3'b001,  c_2   = 3'b010,  c_7   = 3'b011,
              c_22  = 3'b100,  c_24  = 3'b101,  c_26  = 3'b110,  c_m1  = 3'b111;
  
  localparam  e_nn  = 3'b000,  e_au  = 3'b001,  e_t1  = 3'b010,  e_t2  = 3'b011,
              e_t3  = 3'b100,  e_wp  = 3'b101,  e_ea  = 3'b110;
  
  localparam  p_nn  = 3'b000,
              p_au  = 3'b000,  p_pc  = 3'b001,  p_ea  = 3'b010,  p_t1 = 3'b011,
              p_t2  = 3'b100,  p_wp  = 3'b101;  
  
  localparam  no_au = 3'b000,  pc_au = 3'b001,  wp_au = 3'b010,  st_au = 3'b011,
              ea_au = 3'b100,  t1_au = 3'b101,  t2_au = 3'b110,  t3_au = 3'b111;
  
  localparam  no_di = 3'b000,  pc_di = 3'b001,  wp_di = 3'b010,  ir_di = 3'b011,
              ea_di = 3'b100,  t1_di = 3'b101,  t2_di = 3'b110,  t3_di = 3'b111;
  
  localparam  nosh  = 4'b0000, div   = 4'b0001, div2  = 4'b0010, lctr  = 4'b0011,
              mpy   = 4'b0100, stcr  = 4'b0101, stc2  = 4'b0110, alu8  = 4'b0111,
              bus8  = 4'b1000, ldcr  = 4'b1001, sboz  = 4'b1010, shft  = 4'b1011,
              mpys  = 4'b1100, ctr8  = 4'b1101;
            
  localparam  nost  = 4'b0000, st02 = 4'b0001,  st04  = 4'b0010, ldst  = 4'b0011,
              stim  = 4'b0100, stmv = 4'b0101,  stz   = 4'b0110, stovf = 4'b0111,
              stsft = 4'b1000, sttb = 4'b1001,  stxop = 4'b1010, stjmp = 4'b1011,
              stov  = 4'b1100, st4z = 4'b1101,  stdi  = 4'b1110, stic  = 4'b1111;
  
  // Microword decode into functional fields
  `define uword  45:0
  wire [`uword] step;   // step is the current microword

  wire       nmemen = step[45];     // nMEMEN is same as BST3
  wire [3:0] bst    = step[45:42];  // bus cycle status code
  wire [2:0] abus   = step[41:39];  // A bus selector
  wire [3:0] bbus   = step[38:35];  // B bus selector
  wire [2:0] cmux   = step[34:32];  // index for constants table
  wire [3:0] aluop  = step[31:28];  // ALU operation
  wire [2:0] ebus   = step[27:25];  // E bus selector
  wire [2:0] pbus   = step[24:22];  // P bus selector
  wire [2:0] wr_au  = step[21:19];  // ALU output destination
  wire [2:0] wr_di  = step[18:16];  // DB_IN destination
  wire [3:0] ctrl   = step[15:12];  // Misc. control lines
  wire [3:0] stst   = step[11: 8];  // Status register control
  wire [3:0] seqop  = step[ 7: 4];  // Sequencer operation
  wire [3:0] trgt   = step[ 3: 0];  // Sequencer jump target

  // ==========================================================================
  // Bus definitions & bus multiplexers
  // ==========================================================================

  reg [15:0] a_bus, b_bus, e_bus, p_bus;
  
  reg [15:0] ctab[7:0];
  initial
  begin
    ctab[0] =  0; ctab[1] =  1; ctab[2] =  2; ctab[3] = 7;
    ctab[4] = 22; ctab[5] = 24; ctab[6] = 26; ctab[7] = 16'hffff;
  end
  wire [15:0] c_mux = ctab[cmux];

  always @*
  begin
    case (abus)
      a_pc:    a_bus = pc;
      a_ea:    a_bus = ea;
      a_t1:    a_bus = t1;
      a_t2:    a_bus = t2;
      a_t3:    a_bus = t3;
      a_st:    a_bus = st;
      a_c0:    a_bus = 0;
      default: a_bus = wp;
    endcase
  end

  always @*
  begin
    case (bbus)
      b_cb:    b_bus = (byte_ins) ? 16'd1 : 16'd2;
      b_con:   b_bus = c_mux;
      b_t1:    b_bus = t1;
      b_t2:    b_bus = t2;
      b_t3:    b_bus = t3;
      b_cr:    b_bus = { 12'h1ec, ir[7:5], 1'b0 };
      b_xop:   b_bus = { 10'd1, ir[9:6], 2'd0 };
      b_int:   b_bus = { 10'd0, IC[3:0], 2'd0 };
      b_ofs:   b_bus = { {7{ir[7]}}, ir[7:0], 1'b0 };
      b_cc:    b_bus = { 12'd0, ir[9:6] };
      b_sc:    b_bus = { 12'd0, ir[7:4] };
      b_drg:   b_bus = { 11'd0, ir[9:6], 1'b0 };
      default: b_bus = { 11'd0, ir[3:0], 1'b0 }; /* b_srg */
    endcase
  end

  always @*
  begin
    case (ebus)
      e_t1:    e_bus = t1;
      e_t2:    e_bus = t2;
      e_t3:    e_bus = t3;
      e_ea:    e_bus = ea;
      e_wp:    e_bus = wp;
      default: e_bus = a_out;
    endcase
  end
  
  always @*
  begin
    case (pbus)
      p_pc:    p_bus = pc;
      p_ea:    p_bus = ea;
      p_wp:    p_bus = wp;
      p_t1:    p_bus = t1;
      p_t2:    p_bus = t2;
      default: p_bus = a_out;
    endcase
  end

  // ==========================================================================
  // ALU logic
  // ==========================================================================

  reg [15:0] a_out;
  reg alu_lgt, alu_agt, alu_z, alu_c, alu_ovf, alu_po;
  
  always @*
  begin: alu
    reg [16:0] xa, xb, xo;
  
    // calculate result
    xa = (byte_alu) ? { a_bus[15:8], 8'b0 } : a_bus;
    xb = (byte_alu) ? { b_bus[15:8], 8'b0 } : b_bus;
    case (aluop)
      add:           xo = xa + xb;
      sub, neg, cmp: xo = xa - xb;
      ps_a:          xo = xa;
      ps_b:          xo = xb;
      land:          xo = xa & xb;
      lor:           xo = xa | xb;
      landn:         xo = xa & ~xb;
      lxor:          xo = xa ^ xb;
      swap:          xo = { xa[7:0], xa[15:8] };
      abs:           xo = (xa[15]) ? xb - xa : xa;
      default:       xo = xa; /* no-op, but prevent xo latch */
    endcase
    a_out = (byte_alu) ? { xo[15:8], a_bus[7:0] } : xo[15:0];
    
    // set flags
    alu_z  = !(|xo[15:0]);
  
    if (aluop==cmp) begin
      alu_lgt = (xa[15]==xb[15]) ? xo[15] :  xo[16];
      alu_agt = (xa[15]==xb[15]) ? xo[15] : !xo[16];
      alu_po =  (^xa[15:8]);
    end
    else begin
      alu_lgt = !alu_z;
      alu_agt = alu_z ? 1'b0 : ((aluop==abs) ? !xa[15] : !xo[15]);
      alu_po =  (^xo[15:8]);
    end
  
    if (aluop==abs || aluop==neg) begin
      alu_c   = 0;
      alu_ovf = (a_bus==16'h8000);
    end
    else if (aluop==sub) begin
      alu_c   = !xo[16];
      alu_ovf =   (xa[15] ^ xb[15])  & (xo[15] ^ xa[15]);
    end
    else begin
      alu_c   =  xo[16];
      alu_ovf = (!(xa[15] ^ xb[15])) & (xo[15] ^ xa[15]);
    end
  
  end

  // ==========================================================================
  // Hardware registers
  // ==========================================================================

  reg [15:0] pc = 'hfffe, pc_d, wp = 'hfffe, wp_d, st = 0, st_d,
             ea, ea_d, t1, t1_d, t2, t2_d, t3, t3_d;

  reg bit16, bit16_d;
  
  always @(posedge CLKOUT)
  begin
    if (!stall) begin
//      pc <= {pc_d[15:1],1'b0}; wp <= wp_d; st <= st_d;
      pc <= pc_d; wp <= wp_d; st <= st_d;
      ea <= ea_d; t1 <= t1_d; t2 <= t2_d; t3 <= t3_d;
      bit16 <= bit16_d;
    end
  end
  
  // Register load operations
  always @*
  begin
//    pc_d = {pc[15:1],1'b0}; wp_d = wp; ea_d = ea; t2_d = t2;
    pc_d = pc; wp_d = wp; ea_d = ea; t2_d = t2;
    case (wr_au)
      pc_au: pc_d = (stst!=stjmp || sigjmp) ? {a_out[15:1],1'b0} : {pc[15:1],1'b0};  // implement conditional jmp
      wp_au: wp_d = a_out;
      ea_au: ea_d = a_out;
      t2_au: t2_d = a_out;
		default: /* nothing */ ;
    endcase
    case (wr_di)
      pc_di: pc_d = {db_in[15:1],1'b0};
      wp_di: wp_d = db_in;
      ea_di: ea_d = db_in;
      t2_di: t2_d = db_in;
		default: /* nothing */ ;
    endcase
  end

  reg mq_c;

  // Register T1 is the shift register
  always @*
  begin
    t1_d = t1;
    if (wr_au==t1_au) t1_d = a_out;
    if (wr_di==t1_di) t1_d = db_in;
  
    case (ctrl)
  
    stcr:       t1_d = { CRUIN, t1[15:1] };
    stc2, ldcr: t1_d = { 1'b0,  t1[15:1] };
  
    div, div2:  t1_d = { t1[14:0], mq_c };
    mpy, mpys:  t1_d = { mq_c, t1[15:1] };
    
    shft: case (ir[9:8])
          2'b00: t1_d = { t1[15],   t1[15:1] }; // sra
          2'b01: t1_d = {   1'b0,   t1[15:1] }; // srl
          2'b10: t1_d = { t1[14:0],     1'b0 }; // sla
          2'b11: t1_d = { t1[ 0],   t1[15:1] }; // src
          endcase

    default: /* nothing */ ;
    endcase
  end

  wire t1zero  = !(|t1);
  wire sla_ovf = ((ir[9]&!ir[8]) && (t1[15]^t1[14]));

  // keep track of dividend sign for divs, not for div (ir[13]==0/1)
  // and only div-subtract when dividend large enough.
  wire sgn = ir[13] ? 1'b0 : t3[15];
  wire blk = sgn & (bitctr!=0);
  wire dosub1 = (alu_z&!blk) || sgn^alu_c || sgn^bit16;
  wire dosub2 = alu_z        || sgn^alu_c || sgn^bit16;

  // Register T3 is the MQ register
  always @*
  begin
    t3_d = t3; bit16_d = bit16; mq_c = 0;
    if (wr_au==t3_au) t3_d = a_out;
    if (wr_di==t3_di) begin t3_d = db_in; bit16_d = ir[13] ? 1'b0 : db_in[15]; end

    case (ctrl)
  
      div:  { bit16_d, t3_d, mq_c } = dosub1 ? { a_out, t1[15], 1'b1 } : { t3, t1[15], 1'b0 };
      div2: { bit16_d, t3_d, mq_c } = dosub2 ? { 1'b0,  a_out,  1'b1 } : { 1'b0, t3,   1'b0 };
      
      mpy:  { t3_d, mq_c } = t1[0] ? { alu_c,             a_out } : { 1'b0,   t3 };
      mpys: { t3_d, mq_c } = t1[0] ? { a_out[15]^alu_ovf, a_out } : { t3[15], t3 };
  
      default: /* nothing */ ;
    endcase
  end

  // Status register
  always @*
  begin
    st_d = st;
    case (stst)
    ldst:  st_d        = a_out;
    stdi:  st_d        = db_in;
    stim:  st_d[3:0]   = t1[3:0];
    stic:  st_d[3:0]   = (|IC) ? (IC - 1'b1) : 4'b0;
    st04:  st_d[15:11] = { alu_lgt, alu_agt, alu_z, alu_c, alu_ovf };
    st02:  st_d[15:13] = { alu_lgt, alu_agt, alu_z };
    stmv:  st_d[15:13] = { !t1zero, !(t1zero|t1[15]), t1zero };
    stz:   st_d[13]    = alu_z;
    sttb:  st_d[13]    = CRUIN;
    stov:  st_d[11]    = 1'b1;
    st4z:  st_d[11]    = (ir[9:8]==2'b10) ? 1'b0 : st[11];
    stsft: begin
           case (ir[9:8])
           2'b10: begin  // sla
             st_d[12] = t1[15];
             st_d[11] = sla_ovf | st[11];
             end
           default: st_d[12] = t1[0]; // sra, srl, src
           endcase
           end
    stxop: st_d[ 9]    = 1;
    default: /* nothing */ ;
    endcase
    if (byte_alu) st_d[10] = alu_po;
  end

  // Convenience status signals
  wire st_lgt = st[15];
  wire st_agt = st[14];
  wire st_eq  = st[13];
  wire st_c   = st[12];
  wire st_ovf = st[11];
  wire st_op  = st[10];

  // ==========================================================================
  // Instruction prefetcher + instruction decoder
  // ==========================================================================

  // Instruction prefetch
  reg [15:0] ir;
  
  always @(posedge CLKOUT)
  begin
    if (!stall)
      if ((bst==iaq)||(wr_di==ir_di)) ir <= db_in;
    if (RESET)
      ir <= 0;
  end
  
  // Decoder: input is the instruction register, output is:
  reg [7:0] ins, op1, op2;
  reg       nofetch;
  
  localparam none   = 3'd0, do_sop = 3'd1, do_dsop = 3'd2, do_rsop = 3'd3,
             do_reg = 3'd4, do_imm = 3'd5, do_bit  = 3'd6, do_r0sp = 3'd7;
  
  always @*
  begin : decoder
    reg [2:0] ope;       // operand mode
    reg       nf, nfs;   // 'no fetch' optimalisation (clr, seto, mov, etc.)

    nf  = 0; ope = none; op1 = none; op2 = none;
    
    // decode instruction
    casez (ir)
    16'b111?????????????:  begin ins =  31; ope = do_dsop;  nf = 0; end // soc, socb
    16'b110000??????????:  begin ins =  34; ope = do_sop;   nf = 0; end // mov S,R
    16'b1101????????????:  begin ins =  35; ope = do_dsop;  nf = 0; end // movb
    16'b1100????????????:  begin ins =  35; ope = do_dsop;  nf = 1; end // mov
    16'b101?????????????:  begin ins =  29; ope = do_dsop;  nf = 0; end // a, ab
    16'b100?????????????:  begin ins =  33; ope = do_dsop;  nf = 0; end // c, cb
    16'b011?????????????:  begin ins =  30; ope = do_dsop;  nf = 0; end // s, sb
    16'b010?????????????:  begin ins =  32; ope = do_dsop;  nf = 0; end // szc, szcb
    16'b001111??????????:  begin ins =  36; ope = do_sop;   nf = 0; end // div
    16'b001110??????????:  begin ins =  46; ope = do_rsop;  nf = 0; end // mpy
    16'b001101??????????:  begin ins =  50; ope = do_sop;   nf = 0; end // stcr
    16'b001100??????????:  begin ins =  60; ope = do_sop;   nf = 0; end // ldcr
    16'b001011??????????:  begin ins =  65; ope = do_sop;   nf = 1; end // xop
    16'b001010??????????:  begin ins =  74; ope = do_rsop;  nf = 0; end // xor
    16'b001001??????????:  begin ins =  75; ope = do_rsop;  nf = 0; end // czc
    16'b001000??????????:  begin ins =  76; ope = do_rsop;  nf = 0; end // coc
    16'b00011101????????:  begin ins = 121; ope = do_bit;   nf = 0; end // sbz
    16'b00011110????????:  begin ins = 121; ope = do_bit;   nf = 0; end // sbo
    16'b00011111????????:  begin ins = 122; ope = do_bit;   nf = 0; end // tb
    16'b0001????????????:  begin ins = 120; ope = none;     nf = 0; end // relative jumps
//  16'b000011??????????:  undefined (0C00-0CFF)      
    16'b000010??????????:  begin ins = 123; ope = none;     nf = 0; end // src, sla, srl, sra
//  16'b000001111???????:  undefined (0780-07FF)
    16'b0000011101??????:  begin ins = 129; ope = do_sop;   nf = 0; end // abs
    16'b0000011100??????:  begin ins = 130; ope = do_sop;   nf = 1; end // seto
    16'b0000011011??????:  begin ins = 131; ope = do_sop;   nf = 0; end // swpb
    16'b0000011010??????:  begin ins = 134; ope = do_sop;   nf = 1; end // bl
    16'b0000011001??????:  begin ins = 137; ope = do_sop;   nf = 0; end // dect
    16'b0000011000??????:  begin ins = 138; ope = do_sop;   nf = 0; end // dec
    16'b0000010111??????:  begin ins = 139; ope = do_sop;   nf = 0; end // inct
    16'b0000010110??????:  begin ins = 140; ope = do_sop;   nf = 0; end // inc
    16'b0000010101??????:  begin ins = 141; ope = do_sop;   nf = 0; end // inv
    16'b0000010100??????:  begin ins = 142; ope = do_sop;   nf = 0; end // neg
    16'b0000010011??????:  begin ins = 143; ope = do_sop;   nf = 1; end // clr
    16'b0000010010??????:  begin ins = 144; ope = do_sop;   nf = 1; end // x
    16'b0000010001??????:  begin ins = 145; ope = do_sop;   nf = 1; end // b
    16'b0000010000??????:  begin ins = 146; ope = do_sop;   nf = 0; end // blwp
    16'b00000011111?????:  begin ins = 151; ope = none;     nf = 0; end // lrex
    16'b00000011110?????:  begin ins = 151; ope = none;     nf = 0; end // ckof
    16'b00000011101?????:  begin ins = 151; ope = none;     nf = 0; end // ckon
    16'b00000011100?????:  begin ins = 153; ope = none;     nf = 0; end // rtwp
    16'b00000011011?????:  begin ins = 150; ope = none;     nf = 0; end // rset
    16'b00000011010?????:  begin ins = 171; ope = none;     nf = 0; end // idle
//  16'b00000011001?????:  undefined (0320-033F)
    16'b00000011000?????:  begin ins = 160; ope = do_imm;   nf = 1; end // limi
    16'b00000010111?????:  begin ins = 161; ope = do_imm;   nf = 1; end // lwpi
    16'b00000010110?????:  begin ins = 162; ope = do_reg;   nf = 1; end // stst
    16'b00000010101?????:  begin ins = 163; ope = do_reg;   nf = 1; end // stwp
    16'b00000010100?????:  begin ins = 164; ope = do_imm;   nf = 0; end // ci
    16'b00000010011?????:  begin ins = 167; ope = do_imm;   nf = 0; end // ori
    16'b00000010010?????:  begin ins = 166; ope = do_imm;   nf = 0; end // andi
    16'b00000010001?????:  begin ins = 165; ope = do_imm;   nf = 0; end // ai
    16'b00000010000?????:  begin ins = 168; ope = do_imm;   nf = 1; end // li
//  16'b00000001????????:  // undefined (0100-01FF)
    16'b0000000111??????:  begin ins = 200; ope = do_r0sp;  nf = 0; end // mpys  
    16'b0000000110??????:  begin ins = 210; ope = do_r0sp;  nf = 0; end // divs  
    16'b000000001000????:  begin ins = 169; ope = do_reg;   nf = 0; end // lst
    16'b000000001001????:  begin ins = 170; ope = do_reg;   nf = 0; end // lwp
//  16'b0000000?????????:  // undefined (0000-00XX)
    default:               begin ins =   0;                         end // undefined, catch all
    endcase

    nfs = nf; // track whether nofetch applies to destination or source operand 

    // decode operand field(s)
    if (ope==do_dsop) begin
      case (ir[11:10])
      0: op2 = (nf) ? 12 : 13;        // dop_rnf : dop_reg
      1: op2 = 14;                    // dop_ind
      2: op2 = (ir[9:6]==0) ? 18: 19; // dop_sym : dop_idx
      3: op2 = 15;                    // dop_inc
      endcase
      nfs = 0; ope = do_sop;
    end
    if (ope==do_rsop) begin
      op2 = (nf) ? 12 : 13;           // dop_rnf : dop_reg;
      nfs = 0; ope = do_sop;
    end
    if (ope==do_r0sp) begin
      op2 = 23;                       // dop_reg0;
      ope = do_sop;
    end
    
    if (ope==do_sop) begin
      case (ir[5:4])
      0: op1 = (nfs) ? 1 : 2;        // sop_rnf : sop_reg
      1: op1 = 3;                    // sop_ind
      2: op1 = (ir[3:0]==0) ? 7 : 8; // sop_sym : sop_idx
      3: op1 = 4;                    // sop_inc
      endcase
    end
    if (ope==do_reg)
      op1 = (nf) ? 1 : 2;            // sop_rnf : sop_reg;
    if (ope==do_imm)
      op1 = (nf) ? 25 : 24;          // imm : reg_imm;
    if (ope==do_bit)
      op1 = 26;                      // bitcru;

    nofetch = nf;
  end

  // jump condition decoder
  reg sigjmp;

  always @*
  begin
    case (ir[11:8])
     0: sigjmp = 1;                  // jmp
     1: sigjmp = !st_agt & !st_eq;   // jlt
     2: sigjmp = !st_lgt |  st_eq;   // jle
     3: sigjmp =  st_eq;             // jeq
     4: sigjmp =  st_lgt |  st_eq;   // jhe
     5: sigjmp =  st_agt;            // jgt
     6: sigjmp = !st_eq;             // jne
     7: sigjmp = !st_c;              // jnc
     8: sigjmp =  st_c;              // joc
     9: sigjmp = !st_ovf;            // jno
    10: sigjmp = !st_lgt & !st_eq;   // jl
    11: sigjmp =  st_lgt & !st_eq;   // jh
    12: sigjmp =  st_op;             // jop
    default: sigjmp = 0;
    endcase
  end
  
  // decode instructions that can operate on bytes
  reg  byte_ins;
  wire byte_cru, byte_mem;
    
  always @(posedge CLKOUT) if (!stall) byte_ins <= byte_mem | byte_cru;
  
  assign byte_mem = (ir[15]|ir[14]) ? ir[12] : 1'b0;
  
  assign byte_cru = (ir[15:11]==5'b00110) && ((ir[9:6]==4'b1000) || (!ir[9] && (|ir[8:6])));

  // ==========================================================================
  // Sequencer
  // ==========================================================================
  
  // bit counter counts for shift, ldcr/stcr and mpy/div operations
  //
  reg [3:0] bitctr, bitctr_d;
  
  always @(posedge CLKOUT) if (!stall) bitctr <= bitctr_d;

  always @*
  begin
    bitctr_d = bitctr - 1'b1;
    if (ctrl==lctr) bitctr_d = a_out[3:0];
    if (ctrl==ctr8) bitctr_d = 4'b1111;    //8 cycles for SWPB
  end
  
  wire ctr_is0 = !(|bitctr);
  
  // sequencer commands
  localparam srtn = 4'b0000, sget = 4'b0001, drtn = 4'b0010, dget = 4'b0011,
             nins = 4'b0100, goto = 4'b0101, skip = 4'b0110, skhe = 4'b0111,
             skeq = 4'b1000, skne = 4'b1001, wctr = 4'b1010, idle = 4'b1011,
             skle = 4'b1100, nint = 4'b1101;
             
  localparam next  = skip; // 'next' is same as 'skip 0'

  // goto labels and jump table
  localparam ____  = 4'b0000, fin1 = 4'b0001, fin2 = 4'b0010, fin3 = 4'b0011,
             fin4  = 4'b0100, ctx5 = 4'b0101, ctx4 = 4'b0110, fin5 = 4'b0111;

  reg [7:0] jmptab[7:0];
  initial begin
    jmptab[0] =   0; jmptab[1] = 173; jmptab[2] = 174; jmptab[3] = 175;
    jmptab[4] = 176; jmptab[5] = 189; jmptab[6] = 188; jmptab[7] = 193;
  end
  
  // seq is the microprogram sequencer
  //
  reg [7:0] seq = 0, seq_d;
    
  always @(posedge CLKOUT) if (!stall) seq <= seq_d;
  
  wire [7:0] opn = (op2!=none) ? op2 : ins;

  // handle interrupts at end of instruction or during idle
  wire int_ok = (seqop==nins) | (seqop==idle);
  wire nmi_ok = int_ok | (seqop==nint);

  always @*
  begin
    case (seqop)

      // opcode sequencing
      srtn:    seq_d = opn;
      sget:    seq_d = (nofetch & op2==none) ? opn : 8'd11;
      dget:    seq_d = (nofetch) ? ins : 8'd22;
      drtn:    seq_d = ins;
      nint,    // same as nins, but ignore interrupts
      nins:    seq_d = (op1!=none) ? op1 : opn;

      // (conditional) jumps
      goto:    seq_d = jmptab[trgt];
      skip:    seq_d = seq + 8'd1 + trgt;
      skhe:    seq_d = seq + 8'd1 + (( alu_agt|alu_z) ? trgt : 8'd0);
      skle:    seq_d = seq + 8'd1 + ((!alu_agt|alu_z) ? trgt : 8'd0);
      skeq:    seq_d = seq + 8'd1 + (( alu_z)         ? trgt : 8'd0);
      skne:    seq_d = seq + 8'd1 + ((!alu_z)         ? trgt : 8'd0);
      
      wctr:    seq_d = seq + ctr_is0;

      default: seq_d = seq; /* idle */
    endcase
    
    // handle interrupts
    if (int_ok && INT && IC<=st[3:0]) seq_d = 180;
    if (nmi_ok && NMI) seq_d = 177;
    // MID/OVF trap goes here
    if (RESET) seq_d = 182;
  
  end
  
  // enable byte handling as needed
  wire byte_bus = (ctrl==bus8) & (byte_ins);
  wire byte_alu = (ctrl==alu8) & (byte_ins);
  
  reg [`uword] pla[290:0];
  
  assign step = pla[seq];

  initial
  begin
                           /* bst,  abus, bbus,  cnst, alu,   ebus, pbus, wr_au, wr_di, ctrl, sts,   next, trgt */ 

  /* none    */  pla[  0] = 0;

  // Source operand states

  /* sop_rnf */  pla[  1] = { aums, a_wp, b_srg, ___,  add,   e_nn, p_au, ea_au, no_di, nosh, nost,  srtn, ____ };
  /* sop_reg */  pla[  2] = { ws,   a_wp, b_srg, ___,  add,   e_nn, p_au, ea_au, t1_di, nosh, nost,  srtn, ____ };  // ea <- @Rs; t1 <- Rs
  /* sop_ind */  pla[  3] = { ws,   a_wp, b_srg, ___,  add,   e_nn, p_au, no_au, ea_di, nosh, nost,  sget, ____ };
  /* sop_inc */  pla[  4] = { ws,   a_wp, b_srg, ___,  add,   e_nn, p_au, no_au, ea_di, nosh, nost,  next, ____ };
                 pla[  5] = { aums, a_ea, b_cb,  ___,  add,   e_nn, p_au, t1_au, no_di, nosh, nost,  next, ____ };
                 pla[  6] = { ws,   a_wp, b_srg, ___,  add,   e_t1, p_au, no_au, no_di, nosh, nost,  sget, ____ };
  /* sop_sym */  pla[  7] = { iop,  a_pc, b_con, c_2,  add,   e_nn, p_pc, pc_au, ea_di, nosh, nost,  sget, ____ };
  /* sop_idx */  pla[  8] = { ws,   a_wp, b_srg, ___,  add,   e_nn, p_au, no_au, ea_di, nosh, nost,  next, ____ };
                 pla[  9] = { iop,  a_pc, b_con, c_2,  add,   e_nn, p_pc, pc_au, t1_di, nosh, nost,  next, ____ };
                 pla[ 10] = { aums, a_ea, b_t1,  ___,  add,   e_nn, p_nn, ea_au, no_di, nosh, nost,  sget, ____ };
  /* sget    */  pla[ 11] = { sop,  a_nn, b_nn,  ___,  nnnn,  e_nn, p_ea, no_au, t1_di, bus8, nost,  srtn, ____ };

  // Destination operand states

  /* dop_rnf */  pla[ 12] = { aums, a_wp, b_drg, ___,  add,   e_nn, p_au, ea_au, no_di, nosh, nost,  drtn, ____ };
  /* dop_reg */  pla[ 13] = { ws,   a_wp, b_drg, ___,  add,   e_nn, p_au, ea_au, t2_di, nosh, nost,  drtn, ____ };
  /* dop_ind */  pla[ 14] = { ws,   a_wp, b_drg, ___,  add,   e_nn, p_au, no_au, ea_di, nosh, nost,  dget, ____ };
  /* dop_inc */  pla[ 15] = { ws,   a_wp, b_drg, ___,  add,   e_nn, p_au, no_au, ea_di, nosh, nost,  next, ____ };
                 pla[ 16] = { aums, a_ea, b_cb,  ___,  add,   e_nn, p_au, t2_au, no_di, nosh, nost,  next, ____ };
                 pla[ 17] = { ws,   a_wp, b_drg, ___,  add,   e_t2, p_au, no_au, no_di, nosh, nost,  dget, ____ };
  /* dop_sym */  pla[ 18] = { iop,  a_pc, b_con, c_2,  add,   e_nn, p_pc, pc_au, ea_di, nosh, nost,  dget, ____ };
  /* dop_idx */  pla[ 19] = { ws,   a_wp, b_drg, ___,  add,   e_nn, p_au, no_au, ea_di, nosh, nost,  next, ____ };
                 pla[ 20] = { iop,  a_pc, b_con, c_2,  add,   e_nn, p_pc, pc_au, t2_di, nosh, nost,  next, ____ };
                 pla[ 21] = { aums, a_ea, b_t2,  ___,  add,   e_nn, p_nn, ea_au, no_di, nosh, nost,  dget, ____ };
  /* dget    */  pla[ 22] = { dop,  a_nn, b_nn,  ___,  nnnn,  e_nn, p_ea, no_au, t2_di, bus8, nost,  drtn, ____ };
  /* dop_rg0 */  pla[ 23] = { ws,   a_wp, b_con, c_0,  add,   e_nn, p_au, ea_au, t2_di, nosh, nost,  drtn, ____ };  // ea <- @R0; t2 <- R0
  
  // Other operand states: Rx+immediate, immediate and bitcru offset
  
  /* reg_imm */  pla[ 24] = { ws,   a_wp, b_srg, ___,  add,   e_nn, p_au, ea_au, t2_di, nosh, nost,  next, ____ };
  /* imm     */  pla[ 25] = { iop,  a_pc, b_con, c_2,  add,   e_nn, p_pc, pc_au, t1_di, nosh, nost,  srtn, ____ };
  /* bitcru  */  pla[ 26] = { aums, a_wp, b_con, c_24, add,   e_nn, p_nn, ea_au, no_di, nosh, nost,  next, ____ };
                 pla[ 27] = { ws,   a_nn, b_nn,  ___,  nnnn,  e_nn, p_ea, no_au, t1_di, nosh, nost,  next, ____ };
                 pla[ 28] = { aums, a_t1, b_ofs, ___,  add,   e_nn, p_nn, ea_au, no_di, nosh, nost,  srtn, ____ };
  
  // Type I instructions
  
  /* a       */  pla[ 29] = { iaq,  a_t2, b_t1,  ___,  add,   e_nn, p_pc, t1_au, no_di, alu8, st04,  goto, fin4 };  // t1 <- t2 + t1
  /* s       */  pla[ 30] = { iaq,  a_t2, b_t1,  ___,  sub,   e_nn, p_pc, t1_au, no_di, alu8, st04,  goto, fin4 };  // t1 <- t2 - t1
  /* soc     */  pla[ 31] = { iaq,  a_t2, b_t1,  ___,  lor,   e_nn, p_pc, t1_au, no_di, alu8, st02,  goto, fin4 };  // t1 <- t2 | t1
  /* szc     */  pla[ 32] = { iaq,  a_t2, b_t1,  ___,  landn, e_nn, p_pc, t1_au, no_di, alu8, st02,  goto, fin4 };  // t1 <- t2 & ~t1
  /* c       */  pla[ 33] = { iaq,  a_t2, b_t1,  ___,  cmp,   e_nn, p_pc, no_au, no_di, alu8, st02,  goto, fin2 };  // set st0-2 for t2 - t1
  /* mov_wr  */  pla[ 34] = { iaq,  a_wp, b_drg, ___,  add,   e_nn, p_pc, ea_au, no_di, nosh, stmv,  goto, fin4 };  // set st0-2 for t1; ea <- @Rd
  /* mov     */  pla[ 35] = { iaq,  a_t2, b_t1,  ___,  ps_b,  e_nn, p_pc, t1_au, no_di, alu8, st02,  goto, fin4 };  // t1 <- t2
  
  // Type II instructions

  /* div     */  pla[ 36] = { aums, a_nn, b_t1,  ___,  ps_b,  e_nn, p_nn, t2_au, no_di, nosh, nost,  skeq, 4'd8 };  // t2 <- t1, goto ovf if t2==0
                 pla[ 37] = { ws,   a_wp, b_drg, ___,  add,   e_nn, p_au, ea_au, t3_di, nosh, nost,  next, ____ };  // t3 <- Rx
                 pla[ 38] = { ws,   a_ea, b_con, c_2,  add,   e_nn, p_au, no_au, t1_di, nosh, nost,  next, ____ };  // t1 <- Rx+1
                 pla[ 39] = { aums, a_t3, b_t2,  ___,  sub,   e_nn, p_nn, no_au, no_di, nosh, nost,  skhe, 4'd5 };  // test t2>=t3, goto ovf if so
                 pla[ 40] = { aums, a_nn, b_con, c_m1, ps_b,  e_nn, p_nn, no_au, no_di, lctr, nost,  next, ____ };  // ctr <- 15
                 pla[ 41] = { aums, a_t3, b_t2,  ___,  sub,   e_nn, p_nn, no_au, no_di, div,  nost,  wctr, ____ };  // do 16 div steps
                 pla[ 42] = { aums, a_t3, b_t2,  ___,  sub,   e_nn, p_nn, no_au, no_di, div2, nost,  next, ____ };  // capture last quotient bit
                 pla[ 43] = { ws,   a_ea, b_con, c_2,  add,   e_t1, p_ea, ea_au, no_di, nosh, nost,  next, ____ };  // Rx <- t1 (quotient); ea <- ea + 2
                 pla[ 44] = { iaq,  a_t3, b_nn,  ___,  ps_a,  e_nn, p_pc, t1_au, no_di, nosh, nost,  goto, fin4 };  // t1 <- t3 (remainder) + prefetch
  /* div_ovf */  pla[ 45] = { iaq,  a_nn, b_nn,  ___,  nnnn,  e_nn, p_pc, no_au, no_di, nosh, stov,  goto, fin2 };  // set OVF status bit + prefetch

  /* mpy     */  pla[ 46] = { aums, a_nn, b_con, c_0,  ps_b,  e_nn, p_nn, t3_au, no_di, nosh, nost,  next, ____ };  // t3 <- 0
                 pla[ 47] = { aums, a_nn, b_con, c_m1, ps_b,  e_nn, p_nn, no_au, no_di, lctr, nost,  next, ____ };  // ctr <- 15
                 pla[ 48] = { aums, a_t2, b_t3,  ___,  add,   e_nn, p_nn, no_au, no_di, mpy,  nost,  wctr, ____ };  // do 16 mpy steps
                 pla[ 49] = { ws,   a_ea, b_con, c_2,  add,   e_t3, p_ea, ea_au, no_di, nosh, nost,  goto, fin3 };  // Rx <- t3 (MSW); ea <- ea + 2

  /* stcr    */  pla[ 50] = { ws,   a_wp, b_con, c_24, add,   e_nn, p_au, no_au, t2_di, nosh, nost,  next, ____ };  // t2 <- (R12)
                 pla[ 51] = { aums, a_nn, b_cc,  ___,  ps_b,  e_nn, p_nn, no_au, no_di, lctr, nost,  next, ____ };  // ctr <- crucount
                 pla[ 52] = { aums, a_t1, b_nn,  ___,  ps_a,  e_nn, p_nn, t3_au, no_di, nosh, nost,  next, ____ };  // t3 <- t1; use 1 ctr step
                 pla[ 53] = { io,   a_t2, b_con, c_2,  add,   e_nn, p_t2, t2_au, no_di, stcr, nost,  wctr, ____ };  // read cru bits
                 pla[ 54] = { aums, a_nn, b_cc,  ___,  ps_b,  e_nn, p_nn, t2_au, no_di, nosh, nost,  next, ____ };  // t2 <- crucount
                 pla[ 55] = { aums, a_c0, b_t2,  ___,  sub,   e_nn, p_nn, t2_au, no_di, nosh, nost,  next, ____ };  // t2 <- 0 - t2
                 pla[ 56] = { aums, a_t2, b_con, c_7,  land,  e_nn, p_nn, t2_au, no_di, lctr, nost,  next, ____ };  // t2 <- t2 & 7; ctr <- t2 & 7
                 pla[ 57] = { aums, a_nn, b_t2,  ___,  ps_b,  e_nn, p_nn, no_au, no_di, nosh, nost,  skeq, 4'd1 };  // test t2 != 0; skip 1 ctr step
                 pla[ 58] = { aums, a_nn, b_nn,  ___,  nnnn,  e_nn, p_nn, no_au, no_di, stc2, nost,  wctr, ____ };  // zero out remaining bits
                 pla[ 59] = { iaq,  a_t3, b_t1,  ___,  ps_b,  e_nn, p_pc, t1_au, no_di, alu8, st02,  goto, fin4 };  // restore low-order byte + prefetch

  /* ldcr    */  pla[ 60] = { ws,   a_wp, b_con, c_24, add,   e_nn, p_au, ea_au, ea_di, nosh, nost,  next, ____ };  // ea <- (R12)
                 pla[ 61] = { aums, a_nn, b_cc,  ___,  ps_b,  e_nn, p_nn, no_au, no_di, lctr, nost,  next, ____ };  // ctr <- crucount
                 pla[ 62] = { aums, a_t1, b_nn,  ___,  ps_a,  e_nn, p_nn, no_au, no_di, alu8, st02,  next, ____ };  // set status; use 1 ctr step
                 pla[ 63] = { io,   a_ea, b_con, c_2,  add,   e_nn, p_ea, ea_au, no_di, ldcr, nost,  wctr, ____ };  // write cru bits
                 pla[ 64] = { iaq,  a_nn, b_nn,  ___,  nnnn,  e_nn, p_pc, no_au, no_di, nosh, nost,  goto, fin2 };  // non-overlaid prefetch

  /* xop     */  pla[ 65] = { aums, a_pc, b_nn,  ___,  ps_a,  e_nn, p_nn, t2_au, no_di, nosh, nost,  next, ____ };  // t2 <- pc
                 pla[ 66] = { sts,  a_nn, b_con, c_0,  ps_b,  e_nn, p_au, no_au, no_di, nosh, nost,  next, ____ };  // echo ST as all zeroes
                 pla[ 67] = { aums, a_nn, b_xop, ___,  ps_b,  e_nn, p_nn, pc_au, no_di, nosh, nost,  next, ____ };  // pc <- XOP vector address
                 pla[ 68] = { inta, a_pc, b_con, c_2,  add,   e_nn, p_pc, pc_au, t1_di, nosh, nost,  next, ____ };  // t1 <- (pc); pc <- pc + 2  (fetch new WP)
                 pla[ 69] = { aums, a_st, b_nn,  ___,  ps_a,  e_nn, p_nn, t3_au, no_di, nosh, nost,  next, ____ };  // t3 <- st
                 pla[ 70] = { wps,  a_t1, b_nn,  ___,  ps_a,  e_nn, p_au, no_au, no_di, nosh, stxop, next, ____ };  // echo new WP; set ST6
                 pla[ 71] = { ws,   a_t1, b_con, c_22, add,   e_ea, p_au, no_au, no_di, nosh, nost,  next, ____ };  // (t1+22) <- ea (save <src> ea in new R11)
                 pla[ 72] = { aums, a_t1, b_con, c_26, add,   e_nn, p_nn, ea_au, no_di, nosh, nost,  next, ____ };  // ea <- t1 + 26 (address of new R13)
                 pla[ 73] = { inta, a_nn, b_nn,  ___,  nnnn,  e_nn, p_pc, no_au, pc_di, nosh, nost,  goto, ctx5 };  // pc <- (pc); (fetch new PC)

  /* xor     */  pla[ 74] = { iaq,  a_t2, b_t1,  ___,  lxor,  e_nn, p_pc, t1_au, no_di, nosh, st02,  goto, fin4 };  // t1 <- t1 xor t2 + prefetch
  /* czc     */  pla[ 75] = { iaq,  a_t1, b_t2,  ___,  land,  e_nn, p_pc, no_au, no_di, nosh, stz,   goto, fin2 };  // set ST2 for "t2 and t1" + prefetch
  /* coc     */  pla[ 76] = { iaq,  a_t1, b_t2,  ___,  landn, e_nn, p_pc, no_au, no_di, nosh, stz,   goto, fin2 };  // set ST2 for "t2 and not t1" + prefetch

  // Type III instructions (jmp's, sbo, sbz, tb)

  /* rel_jmp */  pla[120] = { aums, a_pc, b_ofs, ___,  add,   e_nn, p_nn, pc_au, no_di, nosh, stjmp, goto, fin1 };  // pc <- pc + offset
  /* cru_sb  */  pla[121] = { io,   a_nn, b_nn,  ___,  nnnn,  e_nn, p_ea, no_au, no_di, sboz, nost,  goto, fin1 };  // set/reset bit at addr ea
  /* cru_tb  */  pla[122] = { io,   a_nn, b_nn,  ___,  nnnn,  e_nn, p_ea, no_au, no_di, nosh, sttb,  goto, fin1 };  // test bit at addr ea
  
  // Type IV instructions (sra, srl, src, sla)
  
  /* shift   */  pla[123] = { ws,   a_wp, b_srg, ___,  add,   e_nn, p_au, ea_au, t1_di, nosh, nost,  next, ____ };  // ea <- address of Rx; t1 <- Rx
                 pla[124] = { aums, a_nn, b_sc,  ___,  ps_b,  e_nn, p_nn, t2_au, no_di, nosh, st4z,  skne, 4'd1 };  // t2 <- shift count, reset ST4, skip next if t2!=0
                 pla[125] = { ws,   a_wp, b_nn,  ___,  ps_a,  e_nn, p_au, no_au, t2_di, nosh, nost,  next, ____ };  // t2 <- R0
                 pla[126] = { aums, a_t2, b_con, c_1,  sub,   e_nn, p_nn, no_au, no_di, lctr, nost,  next, ____ };  // ctr <- bitcount - 1
                 pla[127] = { aums, a_nn, b_nn,  ___,  nnnn,  e_nn, p_nn, no_au, no_di, shft, stsft, wctr, ____ };  // perform shift in t1
                 pla[128] = { iaq,  a_t1, b_nn,  ___,  ps_a,  e_nn, p_pc, no_au, no_di, nosh, st02,  goto, fin4 };  // update ST + prefetch
                 
  // Type V instructions
  
  /* abs     */  pla[129] = { iaq,  a_t1, b_con, c_0,  abs,   e_nn, p_pc, t1_au, no_di, nosh, st04,  goto, fin4 };  // t1 <- abs(t1)     + prefetch
  /* seto    */  pla[130] = { iaq,  a_nn, b_con, c_m1, ps_b,  e_nn, p_pc, t1_au, no_di, nosh, nost,  goto, fin4 };  // t1 <- FFFF        + prefetch
//  /* swap    */  pla[131] = { iaq,  a_t1, b_nn,  ___,  swap,  e_nn, p_pc, t1_au, no_di, nosh, nost,  goto, fin4 };  // t1 <- swap(t1)    + prefetch
  /* swapb   */  pla[131] = { iaq,  a_t1, b_nn,  ___,  swap,  e_nn, p_pc, t1_au, no_di, ctr8, nost,  next, ____ };  // t1 <- swap(t1)    + prefetch
                 pla[132] = { aums, a_nn, b_nn,  ___,  nnnn,  e_nn, p_nn, no_au, no_di, nosh, nost,  wctr, ____ };  // ctr <- 8 cycles
                 pla[133] = { aums, a_nn, b_nn,  ___,  nnnn,  e_nn, p_nn, no_au, no_di, nosh, nost,  goto, fin4 };  // Finish up
  /* bl      */  pla[134] = { aums, a_pc, b_nn,  ___,  ps_a,  e_nn, p_nn, t1_au, no_di, nosh, nost,  next, ____ };  // t1 <- pc
                 pla[135] = { aums, a_ea, b_nn,  ___,  ps_a,  e_nn, p_nn, pc_au, no_di, nosh, nost,  next, ____ };  // pc <- ea
                 pla[136] = { iaq,  a_wp, b_con, c_22, add,   e_nn, p_pc, ea_au, no_di, nosh, nost,  goto, fin4 };  // ea <- address of R11 + prefetch
  /* dect    */  pla[137] = { iaq,  a_t1, b_con, c_2,  sub,   e_nn, p_pc, t1_au, no_di, nosh, st04,  goto, fin4 };  // t1 <- t1 - 2      + prefetch
  /* dec     */  pla[138] = { iaq,  a_t1, b_con, c_1,  sub,   e_nn, p_pc, t1_au, no_di, nosh, st04,  goto, fin4 };  // t1 <- t1 - 1      + prefetch
  /* inct    */  pla[139] = { iaq,  a_t1, b_con, c_2,  add,   e_nn, p_pc, t1_au, no_di, nosh, st04,  goto, fin4 };  // t1 <- t1 + 2      + prefetch
  /* inc     */  pla[140] = { iaq,  a_t1, b_con, c_1,  add,   e_nn, p_pc, t1_au, no_di, nosh, st04,  goto, fin4 };  // t1 <- t1 + 1      + prefetch
  /* inv     */  pla[141] = { iaq,  a_t1, b_con, c_m1, lxor,  e_nn, p_pc, t1_au, no_di, nosh, st02,  goto, fin4 };  // t1 <- t1 ^ FFFF   + prefetch
  /* neg     */  pla[142] = { iaq,  a_c0, b_t1,  ___,  sub,   e_nn, p_pc, t1_au, no_di, nosh, st04,  goto, fin4 };  // t1 <- 0 - t1      + prefetch
  /* clr     */  pla[143] = { iaq,  a_nn, b_con, c_0,  ps_b,  e_nn, p_pc, t1_au, no_di, nosh, nost,  goto, fin4 };  // t1 <- 0           + prefetch
  /* x       */  pla[144] = { sop,  a_pc, b_con, c_2,  sub,   e_nn, p_ea, pc_au, ir_di, nosh, nost,  goto, fin5 };  // pc <- pc - 2      + prefetch ea
  /* b       */  pla[145] = { aums, a_ea, b_nn,  ___,  ps_a,  e_nn, p_nn, pc_au, no_di, nosh, nost,  goto, fin1 };  // pc <- ea
  /* blwp    */  pla[146] = { aums, a_ea, b_con, c_2,  add,   e_nn, p_nn, ea_au, no_di, nosh, nost,  next, ____ };  // ea <- ea + 2
                 pla[147] = { aums, a_pc, b_nn,  ___,  ps_a,  e_nn, p_nn, t2_au, no_di, nosh, nost,  next, ____ };  // t2 <- pc
                 pla[148] = { sop,  a_st, b_nn,  ___,  ps_a,  e_nn, p_ea, t3_au, pc_di, nosh, nost,  goto, ctx4 };  // t3 <- st; pc <- (ea)

  // Type VI instructions
  
  /* idle XX */  pla[149] = { io,   a_nn, b_cr,  ___,  ps_b,  e_nn, p_au, no_au, no_di, sboz, nost,  idle, ____ };  // clock external cru bit + stall
  /* rset    */  pla[150] = { aums, a_nn, b_con, c_0,  ps_b,  e_nn, p_nn, no_au, no_di, nosh, stim,  next, ____ };  // set ST int mask to zero
  /* ext     */  pla[151] = { io,   a_nn, b_cr,  ___,  ps_b,  e_nn, p_au, no_au, no_di, sboz, nost,  next, ____ };  // clock external cru bit
                 pla[152] = { sts,  a_st, b_nn,  ___,  ps_a,  e_au, p_nn, no_au, no_di, nosh, nost,  goto, fin1 };  // echo new status
  
  /* rtwp    */  pla[153] = { aums, a_wp, b_con, c_26, add,   e_nn, p_nn, ea_au, no_di, nosh, nost,  next, ____ };  // ea <- address of R13
                 pla[154] = { ws,   a_ea, b_con, c_2,  add,   e_nn, p_ea, ea_au, wp_di, nosh, nost,  next, ____ };  // wp <- (ea); ea <- ea + 2;
                 pla[155] = { ws,   a_ea, b_con, c_2,  add,   e_nn, p_ea, ea_au, pc_di, nosh, nost,  next, ____ };  // pc <- (ea); ea <- ea + 2;
                 pla[156] = { ws,   a_nn, b_nn,  ___,  nnnn,  e_nn, p_ea, no_au, no_di, nosh, stdi,  next, ____ };  // st <- (ea)
                 pla[157] = { sts,  a_st, b_nn,  ___,  ps_a,  e_nn, p_au, no_au, no_di, nosh, nost,  next, ____ };  // echo new ST
                 pla[158] = { iaq,  a_nn, b_nn,  ___,  nnnn,  e_nn, p_pc, no_au, no_di, nosh, nost,  next, ____ };  // non-overlaid prefetch
                 pla[159] = { wps,  a_pc, b_con, c_2,  add,   e_nn, p_wp, pc_au, no_di, nosh, nost,  nins, ____ };  // echo new WP

  // Type VII instructions
  
  /* limi    */  pla[160] = { iaq,  a_t1, b_nn,  ___,  ps_a,  e_nn, p_pc, no_au, no_di, nosh, stim,  goto, fin2 };  // st[3:0] <- t1[3:0]   + prefetch
  /* lwpi    */  pla[161] = { iaq,  a_t1, b_nn,  ___,  ps_a,  e_nn, p_pc, wp_au, no_di, nosh, nost,  goto, fin2 };  // wp <- t1             + prefetch
  /* stst    */  pla[162] = { iaq,  a_st, b_nn,  ___,  ps_a,  e_nn, p_pc, t1_au, no_di, nosh, nost,  goto, fin4 };  // t1 <- st             + prefetch
  /* stwp    */  pla[163] = { iaq,  a_wp, b_nn,  ___,  ps_a,  e_nn, p_pc, t1_au, no_di, nosh, nost,  goto, fin4 };  // t1 <- wp             + prefetch
  /* ci      */  pla[164] = { iaq,  a_t1, b_t2,  ___,  cmp,   e_nn, p_pc, no_au, no_di, nosh, st02,  goto, fin2 };  // set flags for t1-t2  + prefetch
  /* ai      */  pla[165] = { iaq,  a_t1, b_t2,  ___,  add,   e_nn, p_pc, t1_au, no_di, nosh, st04,  goto, fin4 };  // t1 <- t1 + t2        + prefetch
  /* andi    */  pla[166] = { iaq,  a_t1, b_t2,  ___,  land,  e_nn, p_pc, t1_au, no_di, nosh, st02,  goto, fin4 };  // t1 <- t1 & t2        + prefetch
  /* ori     */  pla[167] = { iaq,  a_t1, b_t2,  ___,  lor,   e_nn, p_pc, t1_au, no_di, nosh, st02,  goto, fin4 };  // t1 <- t1 | t2        + prefetch
  /* li      */  pla[168] = { iaq,  a_wp, b_srg, ___,  add,   e_nn, p_pc, ea_au, no_di, nosh, st02,  goto, fin4 };  // ea <- addr. of Rx    + prefetch
  
  // lst and lwp
  
  /* lst     */  pla[169] = { iaq,  a_t1, b_nn,  ___,  ps_a,  e_nn, p_pc, no_au, no_di, nosh, ldst,  goto, fin4 };  // st <- t1             + prefetch
  /* lwp     */  pla[170] = { iaq,  a_t1, b_nn,  ___,  ps_a,  e_nn, p_pc, wp_au, no_di, nosh, nost,  goto, fin4 };  // wp <- t1             + prefetch

  /* idle    */  pla[171] = { aums, a_pc, b_con, c_2,  add,   e_nn, p_nn, pc_au, no_di, nosh, nost,  next, ____ };  // pc <- pc + 2
  /*         */  pla[172] = { io,   a_nn, b_cr,  ___,  ps_b,  e_nn, p_au, no_au, no_di, sboz, nost,  idle, ____ };  // clock external cru bit + stall

  // Instruction tails, interrupts & reset
  
  /* fin1    */  pla[173] = { iaq,  a_nn, b_nn,  ___,  nnnn,  e_nn, p_pc, no_au, no_di, nosh, nost,  next, ____ };  // non-overlaid prefetch
  /* fin2    */  pla[174] = { aums, a_pc, b_con, c_2,  add,   e_nn, p_nn, pc_au, no_di, nosh, nost,  nins, ____ };  // pc <- pc + 2; next instruction
  /* fin3    */  pla[175] = { iaq,  a_nn, b_nn,  ___,  nnnn,  e_nn, p_pc, no_au, no_di, nosh, nost,  next, ____ };  // non-overlaid prefetch
  /* fin4    */  pla[176] = { dop,  a_pc, b_con, c_2,  add,   e_t1, p_ea, pc_au, no_di, bus8, nost,  nins, ____ };  // pc <- pc + 2; store t1; next instruction
  
  /* NMI     */  pla[177] = { aums, a_c0, b_con, c_2,  sub,   e_nn, p_nn, ea_au, no_di, nosh, nost,  next, ____ };  // ea <-  0 - 2
                 pla[178] = { aums, a_ea, b_con, c_2,  sub,   e_nn, p_nn, ea_au, no_di, nosh, nost,  next, ____ };  // ea <- ea - 2
                 pla[179] = { aums, a_st, b_nn,  ___,  ps_a,  e_nn, p_nn, t3_au, no_di, nosh, nost,  skip, 4'd4 };  // t3 <- st
                 
  /* INT     */  pla[180] = { aums, a_st, b_nn,  ___,  ps_a,  e_nn, p_nn, t3_au, no_di, nosh, nost,  next, ____ };  // t3 <- st
                 pla[181] = { aums, a_nn, b_int, ___,  ps_b,  e_nn, p_nn, ea_au, no_di, nosh, stic,  skip, 4'd2 };  // ea <- ivec; update int. mask from IC[3:0]

  /* RESET   */  pla[182] = { rset, a_st, b_nn,  ___,  ps_a,  e_nn, p_nn, t3_au, no_di, nosh, nost,  next, ____ };  // t3 <- st
                 pla[183] = { rset, a_nn, b_con, c_0,  ps_b,  e_nn, p_nn, ea_au, no_di, nosh, ldst,  next, ____ };  // ea <- 0; st <- 0
  
                 pla[184] = { sts,  a_nn, b_con, c_0,  ps_b,  e_nn, p_au, no_au, no_di, nosh, nost,  next, ____ };  // echo status as all zero
                 pla[185] = { inta, a_pc, b_con, c_2,  sub,   e_nn, p_ea, t2_au, t1_di, nosh, nost,  next, ____ };  // t2 <- pc - 2; t1 <- (ea)
                 pla[186] = { aums, a_ea, b_con, c_2,  add,   e_nn, p_nn, ea_au, no_di, nosh, nost,  next, ____ };  // ea <- ea + 2
                 pla[187] = { inta, a_nn, b_nn,  ___,  nnnn,  e_nn, p_ea, no_au, pc_di, nosh, nost,  next, ____ };  // pc <- (ea)
  /* ctx4    */  pla[188] = { wps,  a_t1, b_con, c_26, add,   e_nn, p_t1, ea_au, no_di, nosh, nost,  next, ____ };  // echo new WP; ea <- addres of new R13
  /* ctx5    */  pla[189] = { ws,   a_ea, b_con, c_2,  add,   e_wp, p_ea, ea_au, no_di, nosh, nost,  next, ____ };  // (ea) <- wp;  ea <- ea + 2
                 pla[190] = { ws,   a_ea, b_con, c_2,  add,   e_t2, p_ea, ea_au, no_di, nosh, nost,  next, ____ };  // (ea) <- pc;  ea <- ea + 2
                 pla[191] = { ws,   a_nn, b_nn,  ___,  nnnn,  e_t3, p_ea, no_au, no_di, nosh, nost,  next, ____ };  // (ea) <- st; 
                 pla[192] = { iaq,  a_t1, b_nn,  ___,  ps_a,  e_nn, p_pc, wp_au, no_di, nosh, nost,  next, ____ };  // wp <- t1 + prefetch
  /* fin5    */  pla[193] = { aums, a_pc, b_con, c_2,  add,   e_nn, p_nn, pc_au, no_di, nosh, nost,  nint, ____ };  // pc = pc + 2; next instruction
  
  //=======
  
  /* mpys    */  pla[200] = { aums, a_nn, b_con, c_0,  ps_b,  e_nn, p_nn, t3_au, no_di, nosh, nost,  next, ____ };  // t3 <- 0
                 pla[201] = { aums, a_c0, b_con, c_2,  sub,   e_nn, p_nn, no_au, no_di, lctr, nost,  next, ____ };  // ctr <- 14
                 pla[202] = { aums, a_t3, b_t2,  ___,  add,   e_nn, p_nn, no_au, no_di, mpys, nost,  wctr, ____ };  // do 15 mpys steps
                 pla[203] = { aums, a_t3, b_t2,  ___,  sub,   e_nn, p_nn, no_au, no_di, mpys, nost,  next, ____ };  // do final (subtractive) mpys step
                 pla[204] = { aums, a_t3, b_nn,  ___,  ps_a,  e_nn, p_nn, no_au, no_di, nosh, st02,  next, ____ };  // set status based on MSW
                 pla[205] = { ws,   a_ea, b_con, c_2,  add,   e_t3, p_ea, ea_au, no_di, nosh, nost,  next, ____ };  // R0 <- t3 (MSW); ea <- ea + 2
                 pla[206] = { iaq,  a_t1, b_t3,  ___,  lor,   e_nn, p_pc, no_au, no_di, nosh, stz,   goto, fin4 };  // fix up EQ bit if LSW!=0 + prefetch
  
  /* divs    */  pla[210] = { aums, a_nn, b_t1,  ___,  ps_b,  e_nn, p_nn, t2_au, no_di, nosh, nost,  skeq, 4'hb };  // t2 <- t1, goto ovf if t2==0
                 pla[211] = { ws,   a_wp, b_nn,  ___,  ps_a,  e_nn, p_au, ea_au, t3_di, nosh, nost,  next, ____ };  // t3 <- R0
                 pla[212] = { ws,   a_ea, b_con, c_2,  add,   e_nn, p_au, no_au, t1_di, nosh, nost,  next, ____ };  // t1 <- R1
                 pla[213] = { aums, a_t3, b_t2,  ___,  lxor,  e_nn, p_nn, ea_au, no_di, nosh, nost,  skhe, 4'd1 };  // test sgn(t2)==sgn(t3), negate divisor if not
                 pla[214] = { aums, a_c0, b_t2,  ___,  sub,   e_nn, p_nn, t2_au, no_di, nosh, nost,  next, ____ };  // t2 <- 0 - t2
                 pla[215] = { aums, a_t3, b_t2,  ___,  sub,   e_nn, p_nn, no_au, no_di, div,  nost,  next, ____ };  // do 1 div step
                 pla[216] = { aums, a_t1, b_con, c_1,  land,  e_nn, p_nn, no_au, no_di, nosh, nost,  skne, 4'd5 };  // test |t3|<|t2|, goto ovf if not
                 pla[217] = { aums, a_c0, b_con, c_2,  sub,   e_nn, p_nn, no_au, no_di, lctr, nost,  next, ____ };  // ctr <- 14
                 pla[218] = { aums, a_t3, b_t2,  ___,  sub,   e_nn, p_nn, no_au, no_di, div,  nost,  wctr, ____ };  // do remaining 15 div steps
                 pla[219] = { aums, a_t3, b_t2,  ___,  sub,   e_nn, p_nn, no_au, no_di, div2, nost,  next, ____ };  // capture last quotient bit
                 pla[220] = { aums, a_ea, b_nn,  ___,  ps_a,  e_nn, p_nn, no_au, no_di, nosh, nost,  skhe, 4'd2 };  // quotient negative?
                 pla[221] = { aums, a_c0, b_t1,  ___,  sub,   e_nn, p_nn, t1_au, no_di, nosh, nost,  skle, 4'd3 };  // t1 <- 0 - t1; ovf if quotient > 0
  /* neg_ovf */  pla[222] = { iaq,  a_nn, b_nn,  ___,  nnnn,  e_nn, p_pc, no_au, no_di, nosh, stov,  goto, fin2 };  // set OVF status bit + prefetch
                 pla[223] = { aums, a_t1, b_con, c_0,  sub,   e_nn, p_nn, no_au, no_di, nosh, nost,  skhe, 4'd1 };  // test t1, ovf if quotient is < 0
  /* pos_ovf */  pla[224] = { iaq,  a_nn, b_nn,  ___,  nnnn,  e_nn, p_pc, no_au, no_di, nosh, stov,  goto, fin2 };  // set OVF status bit + prefetch
                 pla[225] = { ws,   a_wp, b_con, c_2,  add,   e_t1, p_wp, ea_au, no_di, nosh, stmv,  next, ____ };  // R0 <- t1 (quotient); ea <- wp + 2
                 pla[226] = { iaq,  a_t3, b_nn,  ___,  ps_a,  e_nn, p_pc, t1_au, no_di, nosh, nost,  goto, fin4 };  // t1 <- t3 (remainder) + prefetch

  end

endmodule
