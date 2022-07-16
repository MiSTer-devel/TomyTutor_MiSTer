// Original code by Paul Ruiz's (gitlab.com/pnru)
// Flandango: minor modification for CPU clock speed of Tomy Tutor and added event based decrementer
module UTIL9995(
  input  wire        clk,
  input  wire        rst,
  
  input  wire [15:0] ab,
  input  wire [15:0] din,
  output wire [15:0] dout,
  input  wire        nmemen,
  input  wire        nwr,
  output wire        utl_sel,
  
  input  wire        cruclk,
  input  wire        cruout,
  output wire        cruin,
  
  input  wire        int1,
  input  wire        int4,
  output reg         irq,
  output reg   [3:0] ic,
  input  wire  [3:0] bst
);

  wire crusel = ab[15:5]==11'b0001_1110_111; // 0x1ee0-0x1eff
  assign utl_sel = (ab==16'hfffa) & !nmemen;   // 0xfffa

  // flag register
  //
  reg [15:0] flag;

  always @(posedge clk) begin
    if (rst)
      flag <= 16'h0000;
    else begin
      if (crusel & !cruclk) flag[ab[4:1]] <= cruout;
		flag[3] <= int3i;
		flag[4] <= int4i;
	 end
  end
  assign cruin = flag[ab[4:1]];
  
  // decrementer
  //
  reg [15:0] start = 0, decr = 0;
//  reg  [3:0] scale = 0; // prescaler for 25Mhz CPU clock
  reg  [1:0] scale = 0; // prescaler for 10.7Mhz CPU clock
  
  always @(posedge clk) scale <= scale + 1'b1;
  wire tick = &scale;
  
  always @(posedge clk) begin
	 if(!flag[0] && flag[1]) if (tick) decr <= decr - 1'b1;
	 else if(flag[0] && flag[1] && decevent) decr <= decr - 1'b1; 

	 if( decr==0 && tick ) begin
		decr <= start;
	 end
    if( utl_sel & !nwr ) begin
		start <= din;
		decr <= din;
	 end
  end
  wire zero = !(|decr);

  assign dout = decr;
  
  // internal INT4 latch
  reg  int1i = 0;
  wire inta1 = (bst==4'b0101) & (ab[5:2]==4'h1);
  
  always @(posedge clk) begin
    if( rst | inta1 )  int1i <= 1'b0;
    if( int1 )         int1i <= 1'b1;
  end
  
  // internal INT3 latch
  reg  int3i = 0;
  wire inta3 = (bst==4'b0101) & (ab[5:2]==4'h3);
  
  always @(posedge clk) begin
    if( zero & flag[1]) int3i <= 1'b1;
    if( rst | inta3 )   int3i <= 1'b0;
  end
  
  // internal INT4 latch
  reg  int4i = 0;
  wire inta4 = (bst==4'b0101) & (ab[5:2]==4'h4);
  reg  decevent = 1'b0;

  always @(posedge clk) begin
    decevent <= 1'b0;
    if( rst | inta4 )  int4i <= 1'b0;
    if( int4 )         int4i <= 1'b1;
	 if( int4 && ~int4i ) decevent <= 1'b1;
  end

  // TMS9995 interrupt encoder
  //
  always @(posedge clk) begin
    ic = 4'hf;
    if( int4i ) ic = 4'h4;
    if( int3i ) ic = 4'h3;
    if( int1i ) ic = 4'h1;
    irq = (ic != 4'hf);
  end
  
endmodule
