
//Avoid giving speed as h10 for the sake of motor
//Here en is different than multiplication. When en=0, outputs are reset to zero. When en=1, normal PWM signal.
//clk defines the time period of the PWM signal
//speed is signed. So if speed[4]=1, the actual speed is negative with value being two's complement
//For the sake of positive and negative speeds, two outputs are provided, namely outp and outn
module PWM_timer(input clk, input [8:0] speed, input en, output out, output dirp, dirn);

	reg [8:0] speed_copy; initial speed_copy<=0;

	reg outlogic; initial outlogic<=0;//This is the output of the logic circuit. It has to be multiplexed with 0 depending on en
	assign out = (en ? outlogic : 0); //If en is off, outp=0
	
	reg [7:0] count; initial count<=0;//Keeping a count of the cycles elapsed

	always @(negedge count[7]) speed_copy <= (speed[8] ? ~speed[7:0]+1 : speed[7:0]); //Latching speed

	always @(negedge clk) begin
		count<=count+1;
		if (count==0 && speed_copy[7:0]!=0) begin
			outlogic<=1;
		end else if (count==speed_copy[7:0]) begin
			outlogic<=0;
		end
	end
	
	assign dirp = (~speed_copy[8]);
	assign dirn = (speed_copy[8]);

endmodule

/*
module PWM_test;

	reg [4:0] speed; reg clk; reg en; wire outp, outn;

	initial begin
		$dumpfile("PWM.vcd");
		$dumpvars(0,PWM_test);
		clk<=0;
		en<=1;
		speed<=0;
		#3000 en<=0;
		#150 $finish;
	end

	always #1 clk<=~clk;

	always #180 speed<=speed+2;
	PWM_timer testPWM(clk,speed,en,outp,outn);

endmodule
*/

