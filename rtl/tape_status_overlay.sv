// Tape Status display for Tomy Tutor by Flandango

module tape_status_overlay
#(parameter X=0,  // Starting Horizontal Pixel
  parameter Y=0,   // Starting Line
  parameter W=75,   // Width in Pixels
  parameter H=11    // Height in Pixels/Scan Lines
)
(
  // Port A
  input                  clk,    // Video Clock
  input                  reset,
  input                  en,
  input        [32:0]    data,   // 19:16 = Activity (play/stop/rewind...), 15:12 = Current Index Digit 1, 11:8 = Current Index Digit 2, 7:4 = Max Index Digit 1, 3:0 = Max Index Digit 2
  input signed [10:0]    num_pix,
  input signed [10:0]    num_line,
  input         [7:0]    r_i,
  input         [7:0]    g_i,
  input         [7:0]    b_i,
  output        [7:0]    r_o,
  output        [7:0]    g_o,
  output        [7:0]    b_o
);

reg [7:0] charset[0:199];
initial begin
	$readmemh("rtl/font.hex", charset);
end

wire in_box =  (num_pix >=X && num_pix < X+W) && (num_line >=Y && num_line < Y+H); // Box in upper left corner of border....for future tape status
reg [7:0] r,g,b;
reg [3:0] tape_activity, cIdx_1, cIdx_2, cIdx_3, mIdx_1, mIdx_2, mIdx_3;
reg [7:0] tapeDsp,cd1,cd2,cd3,md1,md2,md3,lock;
reg       rec_en;

assign r_o           = r;
assign g_o           = g;
assign b_o           = b;

always @(posedge clk) begin
	reg signed [10:0] curPixel;
	reg signed [10:0] curLine;
	curPixel      = num_pix - X[10:0];
	curLine       = num_line - Y[10:0];
	rec_en        = data[28];
   tape_activity = data[27:24];
   cIdx_1        = data[23:20];
   cIdx_2        = data[19:16];
   cIdx_3        = data[15:12];
   mIdx_1        = data[11:8];
   mIdx_2        = data[7:4];
   mIdx_3        = data[3:0];
	
	if(in_box && en) begin
		tapeDsp = curLine == 11'd1 ? 8'd0 : tape_activity == 4'd0 ? 8'd80 : charset[((tape_activity+4'd11)*4'd8)+(curLine-10'd2)];
		cd1 = curLine == 11'd1 ? 8'd0 : charset[cIdx_1*4'd8+(curLine-10'd2)];
		cd2 = curLine == 11'd1 ? 8'd0 : charset[cIdx_2*4'd8+(curLine-10'd2)];
		cd3 = curLine == 11'd1 ? 8'd0 : charset[cIdx_3*4'd8+(curLine-10'd2)];
		md1 = curLine == 11'd1 ? 8'd0 : charset[mIdx_1*4'd8+(curLine-10'd2)];
		md2 = curLine == 11'd1 ? 8'd0 : charset[mIdx_2*4'd8+(curLine-10'd2)];
		md3 = curLine == 11'd1 ? 8'd0 : charset[mIdx_3*4'd8+(curLine-10'd2)];
		lock = curLine == 11'd1 ? 8'd0 : charset[((8'd23 + (rec_en ? 8'd1 : 8'd0))*8'd8)+(curLine-10'd2)];

		if (curLine == 0 || curLine == H-1'b1) begin
				r = 'hFF;
				g = 'hFF;
				b = 'hFF;
		end
		else begin
			if(curPixel == 0 || curPixel == W-1'b1) begin
				r = 'hFF;
				g = 'hFF;
				b = 'hFF;
			end
			else begin
				r = 'h00;
				g = 'h00;
				b = 'h00;
				if( curPixel > 0 && curPixel < 9 ) begin
					if(tapeDsp[7-(curPixel-1'b1)]) begin
					   case(tape_activity)
					      4'd1:  g = 'hFF;
							4'd10: r = 'hFF;
							default: b='hFF;
						endcase
					end
					else {r,g,b} = 24'hFFFF00;
				end
				if(curPixel == 9) {r,g,b} = 24'hFFFF00;				//Yellow line seperating Tape Activity with Digits
				//Current Tape Counter Digit 1 of 3
				else if( curPixel >= 10 && curPixel <= 17 ) begin
				   if(cd1[7-(curPixel-8'd10)]) begin
						{r,g,b} = rec_en ? 24'hFFFFFF : 24'hBBC2C8;
					end
				end
				//Current Tape Counter Digit 2 of 3
				else if( curPixel >= 18 && curPixel <= 25 ) begin
				   if(cd2[7-(curPixel-8'd18)]) begin
						{r,g,b} = rec_en ? 24'hFFFFFF : 24'hBBC2C8;
					end
				end
				//Current Tape Counter Digit 3 of 3
				else if( curPixel >= 26 && curPixel <= 33 ) begin
				   if(cd3[7-(curPixel-8'd26)]) begin
						{r,g,b} = rec_en ? 24'hFFFFFF : 24'hBBC2C8;
					end
				end
				
				// Separator Character (/)
				else if( curPixel >= 34 && curPixel <= 41 ) begin
				   if(charset[88+(curLine-10'd2)][7-(curPixel-8'd34)]) begin
						{r,g,b} = 24'hFFFFFF;
					end
				end
				
				// Maximum Tape Counter Digit 1 of 3
				else if( curPixel >= 42 && curPixel <= 49 ) begin
				   if(md1[7-(curPixel-8'd42)]) begin
						{r,g,b} = rec_en ? 24'hFFFFFF : 24'hBBC2C8;
					end
				end
				// Maximum Tape Counter Digit 2 of 3
				else if( curPixel >= 50 && curPixel <= 57 ) begin
				   if(md2[7-(curPixel-8'd50)]) begin
						{r,g,b} = rec_en ? 24'hFFFFFF : 24'hBBC2C8;
					end
				end
				// Maximum Tape Counter Digit 3 of 3
				else if( curPixel >= 58 && curPixel <= 65 ) begin
				   if(md3[7-(curPixel-8'd58)]) begin
						{r,g,b} = rec_en ? 24'hFFFFFF : 24'hBBC2C8;
					end
				end
				// Write protect Indicator
				else if( curPixel >= 66 && curPixel <= 73 ) begin
				   if(lock[7-(curPixel-8'd66)]) begin
						{r,g,b} = rec_en ? 24'h00FF00 : 24'hFF0000;
					end
				end
			end
		end
	end
	else begin
		r = r_i;
		g = g_i;
		b = b_i;
	end
end
   
endmodule

module bram #(
    parameter width_a = 8,
    parameter widthad_a = 10,
    parameter init_file= ""
) (
    // Port A
    input   wire                clock_a,
    input   wire                wren_a,
    input   wire    [widthad_a-1:0]  address_a,
    input   wire    [width_a-1:0]  data_a,
    output  reg     [width_a-1:0]  q_a,
     
    // Port B
    input   wire                clock_b,
    input   wire                wren_b,
    input   wire    [widthad_a-1:0]  address_b,
    input   wire    [width_a-1:0]  data_b,
    output  reg     [width_a-1:0]  q_b,

    input wire byteena_a,
    input wire byteena_b,
    input wire enable_a,
    input wire enable_b 
);

    initial begin
        $display("Loading rom.");
        $display(init_file);
        if (init_file>0)
        	$readmemh(init_file, mem);
    end

 
// Shared memory
reg [width_a-1:0] mem [(2**widthad_a)-1:0];

// Port A
always @(posedge clock_a) begin
    q_a      <= mem[address_a];
    if(wren_a) begin
        q_a      <= data_a;
        mem[address_a] <= data_a;
    end
end
 
// Port B
always @(posedge clock_b) begin
    q_b      <= mem[address_b];
    if(wren_b) begin
        q_b      <= data_b;
        mem[address_b] <= data_b;

        $display("writingb: %x %x",address_b,data_b);
    end
end
 
endmodule
