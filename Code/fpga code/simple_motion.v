module simple_move (input clk, input pwm_clk,input en_PWM, input rx, output tx, output rx_flag_out, output [7:0] rx_reg_out, output [3:0] sig_to_mot, dirp, dirn);

	parameter FOR="f"; parameter BCK="b"; parameter RHT="r"; parameter LFT="l"; 
	parameter UPW = "u"; parameter DNW = "d"; parameter STOP = "s";
	parameter SPEED_MAX_PULL = 9'h02; parameter SPEED_MAX_PUSH = 9'h80;
	parameter SPEED_UP = 9'h1F; parameter SPEED_DN = 9'hFF; 
	wire clk_PWM, clk_uart;
	assign {clk_PWM, clk_uart} = {pwm_clk, clk};

	reg [8:0] speed0, speed1, speed2, speed3;
	PWM_timer mot0_PWM(clk_PWM, speed0, en_PWM, sig_to_mot[0], dirp[0], dirn[0]);
	PWM_timer mot1_PWM(clk_PWM, speed1, en_PWM, sig_to_mot[1], dirp[1], dirn[1]);
	PWM_timer mot2_PWM(clk_PWM, speed2, en_PWM, sig_to_mot[2], dirp[2], dirn[2]);
	PWM_timer mot3_PWM(clk_PWM, speed3, en_PWM, sig_to_mot[3], dirp[3], dirn[3]);
	
	wire rst, rx_flag, rx_busy, tx_busy, rx_error;
	reg [7:0] tx_reg; wire [7:0] rx_reg;
	reg tx_en;
	uart COM(clk_uart, rst, rx, tx, tx_en, tx_reg, rx_flag, rx_reg, rx_busy, tx_busy, rx_error);
	
	assign {rx_flag_out, rx_reg_out}={rx_flag,rx_reg};
	
	always @(negedge rx_flag) begin
	
		case (rx_reg)
			"f": begin
				speed0<=SPEED_MAX_PULL; speed1<=SPEED_MAX_PULL;
				speed2<=SPEED_MAX_PUSH; speed3<=SPEED_MAX_PUSH;
			end
			"b": begin
				speed0<=SPEED_MAX_PUSH; speed1<=SPEED_MAX_PUSH;
				speed2<=SPEED_MAX_PULL; speed3<=SPEED_MAX_PULL;
			end
			"r": begin
				speed0<=SPEED_MAX_PUSH; speed1<=SPEED_MAX_PULL;
				speed2<=SPEED_MAX_PULL; speed3<=SPEED_MAX_PUSH;
			end
			"l": begin
				speed0<=SPEED_MAX_PULL; speed1<=SPEED_MAX_PUSH;
				speed2<=SPEED_MAX_PUSH; speed3<=SPEED_MAX_PULL;
			end
			"u": begin
				speed0<=SPEED_MAX_PULL; speed1<=SPEED_MAX_PULL;
				speed2<=SPEED_MAX_PULL; speed3<=SPEED_MAX_PULL;
			end
			"d": begin
				speed0<=SPEED_DN; speed1<=SPEED_DN;
				speed2<=SPEED_DN; speed3<=SPEED_DN;
			end
			"s": begin
				speed0<=0; speed1<=0;
				speed2<=0; speed3<=0;
			end
		endcase

	end
	
	
endmodule
