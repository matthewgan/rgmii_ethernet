`timescale 1ns / 1ps
//////////////////////////////////////////////////////////////////////////////////
// Company:             Xunitek
// Engineer:            Devin
// Email:               Devin.song@xunitek.com      	     
// Module Name:         
// HDL Type:            Verilog HDL 
// Target Devices:      zynq-xc7z020clg484-1
// Tool versions:       vivado 2019.1 
// Description:         
// Create Date: 
// Dependencies: 
//--------------------------------------------------------------------------------
// Revision History 		                         
//
//--------------------------------------------------------------------------------
// Additional Comments: 
//protocal data header description:(16 bytes)
//byte          description          type           unit     value
// 4            pkg_length           uint_32        byte 
// 4            pkg_cnt              uint_32        num
// 4            reserved             uint_32        num
// 4            instruct_ID          uint_32        num
//
//instruct_ID:
//001:connect request
//002:disconnect request
//100:platform DOF command mode data
//101:actuator position command mode data
//102:motion cueing mode data
//103:playback mode data
//104:extended motion cueing mode data
//105:actuator position,velocity,acceleration command mode data
// 
//////////////////////////////////////////////////////////////////////////////////
module actuator_pva #(parameter INS_WIDTH = 32)
        (
            input                   i_sys_clk,
            input                   i_rst_n,
            input                   i_act_pva_data_en,                
            input [INS_WIDTH-1:0]   i_act_pva_data,    

            output [INS_WIDTH-1:0]  o_act_pva_axis_A_pos_data,
            output [INS_WIDTH-1:0]  o_act_pva_axis_A_vel_data,
            output [INS_WIDTH-1:0]  o_act_pva_axis_A_acc_data,
            output [INS_WIDTH-1:0]  o_act_pva_axis_B_pos_data,
            output [INS_WIDTH-1:0]  o_act_pva_axis_B_vel_data,
            output [INS_WIDTH-1:0]  o_act_pva_axis_B_acc_data,
            output [INS_WIDTH-1:0]  o_act_pva_axis_C_pos_data,
            output [INS_WIDTH-1:0]  o_act_pva_axis_C_vel_data,
            output [INS_WIDTH-1:0]  o_act_pva_axis_C_acc_data,
            output [INS_WIDTH-1:0]  o_act_pva_axis_D_pos_data,
            output [INS_WIDTH-1:0]  o_act_pva_axis_D_vel_data,
            output [INS_WIDTH-1:0]  o_act_pva_axis_D_acc_data,
            output [INS_WIDTH-1:0]  o_act_pva_axis_E_pos_data,
            output [INS_WIDTH-1:0]  o_act_pva_axis_E_vel_data,
            output [INS_WIDTH-1:0]  o_act_pva_axis_E_acc_data,
            output [INS_WIDTH-1:0]  o_act_pva_axis_F_pos_data,
            output [INS_WIDTH-1:0]  o_act_pva_axis_F_vel_data,
            output [INS_WIDTH-1:0]  o_act_pva_axis_F_acc_data,
            output reg              o_act_pva_axis_data_en
        );

//axis A
reg [INS_WIDTH-1:0]     s_axis_A_pos_max=32'h42C8_0000;//100m
reg [INS_WIDTH-1:0]     s_axis_A_vel_max=32'h42C8_0000;//100m/s
reg [INS_WIDTH-1:0]     s_axis_A_acc_min=32'hC2C8_0000;//-100m/s2
reg [INS_WIDTH-1:0]     s_axis_A_acc_max=32'h42C8_0000;//100m/s2
//axis B
reg [INS_WIDTH-1:0]     s_axis_B_pos_max=32'h42C8_0000;//100m
reg [INS_WIDTH-1:0]     s_axis_B_vel_max=32'h42C8_0000;//100m/s
reg [INS_WIDTH-1:0]     s_axis_B_acc_min=32'hC2C8_0000;//-100m/s2
reg [INS_WIDTH-1:0]     s_axis_B_acc_max=32'h42C8_0000;//100m/s2
//axis C
reg [INS_WIDTH-1:0]     s_axis_C_pos_max=32'h42C8_0000;//100m
reg [INS_WIDTH-1:0]     s_axis_C_vel_max=32'h42C8_0000;//100m/s
reg [INS_WIDTH-1:0]     s_axis_C_acc_min=32'hC2C8_0000;//-100m/s2
reg [INS_WIDTH-1:0]     s_axis_C_acc_max=32'h42C8_0000;//100m/s2
//axis D
reg [INS_WIDTH-1:0]     s_axis_D_pos_max=32'h42C8_0000;//100m
reg [INS_WIDTH-1:0]     s_axis_D_vel_max=32'h42C8_0000;//100m/s
reg [INS_WIDTH-1:0]     s_axis_D_acc_min=32'hC2C8_0000;//-100m/s2
reg [INS_WIDTH-1:0]     s_axis_D_acc_max=32'h42C8_0000;//100m/s2
//axis E
reg [INS_WIDTH-1:0]     s_axis_E_pos_max=32'h42C8_0000;//100m
reg [INS_WIDTH-1:0]     s_axis_E_vel_max=32'h42C8_0000;//100m/s
reg [INS_WIDTH-1:0]     s_axis_E_acc_min=32'hC2C8_0000;//-100m/s2
reg [INS_WIDTH-1:0]     s_axis_E_acc_max=32'h42C8_0000;//100m/s2
//axis F
reg [INS_WIDTH-1:0]     s_axis_F_pos_max=32'h42C8_0000;//100m
reg [INS_WIDTH-1:0]     s_axis_F_vel_max=32'h42C8_0000;//100m/s
reg [INS_WIDTH-1:0]     s_axis_F_acc_min=32'hC2C8_0000;//-100m/s2
reg [INS_WIDTH-1:0]     s_axis_F_acc_max=32'h42C8_0000;//100m/s2

reg [INS_WIDTH-1:0]     s_axis_A_pos_data;
reg [INS_WIDTH-1:0]     s_axis_A_vel_data;
reg [INS_WIDTH-1:0]     s_axis_A_acc_data;

reg [INS_WIDTH-1:0]     s_axis_B_pos_data;
reg [INS_WIDTH-1:0]     s_axis_B_vel_data;
reg [INS_WIDTH-1:0]     s_axis_B_acc_data;

reg [INS_WIDTH-1:0]     s_axis_C_pos_data;
reg [INS_WIDTH-1:0]     s_axis_C_vel_data;
reg [INS_WIDTH-1:0]     s_axis_C_acc_data;

reg [INS_WIDTH-1:0]     s_axis_D_pos_data;
reg [INS_WIDTH-1:0]     s_axis_D_vel_data;
reg [INS_WIDTH-1:0]     s_axis_D_acc_data;

reg [INS_WIDTH-1:0]     s_axis_E_pos_data;
reg [INS_WIDTH-1:0]     s_axis_E_vel_data;
reg [INS_WIDTH-1:0]     s_axis_E_acc_data;

reg [INS_WIDTH-1:0]     s_axis_F_pos_data;
reg [INS_WIDTH-1:0]     s_axis_F_vel_data;
reg [INS_WIDTH-1:0]     s_axis_F_acc_data;



reg [4:0]               s_data_cnt;
reg [1:0]               s_data_loop_cnt;
reg [3:0]               s_axis_cnt;
reg [1:0]               s_data_loop_cnt_reg;
reg [3:0]               s_axis_cnt_reg;
reg [4:0]               s_data_valid_record;
reg                     s_pos_data_valid;
reg                     s_vel_data_valid;
reg                     s_acc_data_valid;
reg [INS_WIDTH-1:0]     s_act_pva_data;
reg                     s_act_pva_data_en;


always@(posedge i_sys_clk or negedge i_rst_n)begin
    if(!i_rst_n)begin
        s_data_loop_cnt <= 'd0;
        s_axis_cnt      <= 'd0;
    end else if(i_act_pva_data_en)begin
        if(s_data_loop_cnt == 2)begin
            s_data_loop_cnt <= 'd0;
            s_axis_cnt      <= s_axis_cnt + 1'b1;
        end else begin
            s_data_loop_cnt <= s_data_loop_cnt + 1'b1;
            s_axis_cnt      <= s_axis_cnt;
        end
    end else begin
        s_data_loop_cnt <= 'd0;
        s_axis_cnt      <= 'd0;
    end
end

always@(posedge i_sys_clk or negedge i_rst_n)begin
    if(!i_rst_n)begin
        s_data_loop_cnt_reg     <=  'd0;
        s_axis_cnt_reg          <=  'd0;
        s_act_pva_data          <=  'd0;
        s_act_pva_data_en       <= 1'b0;
    end else begin
        s_data_loop_cnt_reg     <= s_data_loop_cnt;
        s_axis_cnt_reg          <= s_axis_cnt;
        s_act_pva_data          <= i_act_pva_data;
        s_act_pva_data_en       <= i_act_pva_data_en;
    end
end

always@(posedge i_sys_clk or negedge i_rst_n)begin
    if(!i_rst_n)begin
        s_pos_data_valid    <= 1'b0;
        s_vel_data_valid    <= 1'b0;
        s_acc_data_valid    <= 1'b0;
    end else if(i_act_pva_data_en)begin
        case(s_axis_cnt)
            'd0://axis A
                begin
                    case(s_data_loop_cnt)
                        'd0:
                            begin
                                if(i_act_pva_data[31]==1'b0 && i_act_pva_data<= s_axis_A_pos_max)begin
                                    s_pos_data_valid    <= 1'b1;
                                end else begin
                                    s_pos_data_valid    <= 1'b0;
                                end
                            end
                        'd1:
                            begin
                                if(i_act_pva_data[31]==1'b0 && i_act_pva_data<= s_axis_A_vel_max)begin
                                    s_vel_data_valid    <= 1'b1;
                                end else begin
                                    s_vel_data_valid    <= 1'b0;
                                end
                            end
                        'd2:
                            begin
                                if(i_act_pva_data[31]==1'b0 && i_act_pva_data<= s_axis_A_acc_max)begin
                                    s_acc_data_valid    <= 1'b1;
                                end else if(i_act_pva_data[31]==1'b1 && i_act_pva_data<= s_axis_A_acc_min)begin
                                    s_acc_data_valid    <= 1'b1;
                                end else begin
                                    s_acc_data_valid    <= 1'b0;
                                end
                            end
                        default: ;
                    endcase        
                end
            'd1: //axis B
                begin
                    case(s_data_loop_cnt)
                        'd0:
                            begin
                                if(i_act_pva_data[31]==1'b0 && i_act_pva_data<= s_axis_B_pos_max)begin
                                    s_pos_data_valid    <= 1'b1;
                                end else begin
                                    s_pos_data_valid    <= 1'b0;
                                end
                            end
                        'd1:
                            begin
                                if(i_act_pva_data[31]==1'b0 && i_act_pva_data<= s_axis_B_vel_max)begin
                                    s_vel_data_valid    <= 1'b1;
                                end else begin
                                    s_vel_data_valid    <= 1'b0;
                                end
                            end
                        'd2:
                            begin
                                if(i_act_pva_data[31]==1'b0 && i_act_pva_data<= s_axis_B_acc_max)begin
                                    s_acc_data_valid    <= 1'b1;
                                end else if(i_act_pva_data[31]==1'b1 && i_act_pva_data<= s_axis_B_acc_min)begin
                                    s_acc_data_valid    <= 1'b1;
                                end else begin
                                    s_acc_data_valid    <= 1'b0;
                                end
                            end
                        default: ;
                    endcase        
                end            
            'd2: //axis C
                begin
                    case(s_data_loop_cnt)
                        'd0:
                            begin
                                if(i_act_pva_data[31]==1'b0 && i_act_pva_data<= s_axis_C_pos_max)begin
                                    s_pos_data_valid    <= 1'b1;
                                end else begin
                                    s_pos_data_valid    <= 1'b0;
                                end
                            end
                        'd1:
                            begin
                                if(i_act_pva_data[31]==1'b0 && i_act_pva_data<= s_axis_C_vel_max)begin
                                    s_vel_data_valid    <= 1'b1;
                                end else begin
                                    s_vel_data_valid    <= 1'b0;
                                end
                            end
                        'd2:
                            begin
                                if(i_act_pva_data[31]==1'b0 && i_act_pva_data<= s_axis_C_acc_max)begin
                                    s_acc_data_valid    <= 1'b1;
                                end else if(i_act_pva_data[31]==1'b1 && i_act_pva_data<= s_axis_C_acc_min)begin
                                    s_acc_data_valid    <= 1'b1;
                                end else begin
                                    s_acc_data_valid    <= 1'b0;
                                end
                            end
                        default: ;
                    endcase        
                end  
            'd3: //axis D
                begin
                    case(s_data_loop_cnt)
                        'd0:
                            begin
                                if(i_act_pva_data[31]==1'b0 && i_act_pva_data<= s_axis_D_pos_max)begin
                                    s_pos_data_valid    <= 1'b1;
                                end else begin
                                    s_pos_data_valid    <= 1'b0;
                                end
                            end
                        'd1:
                            begin
                                if(i_act_pva_data[31]==1'b0 && i_act_pva_data<= s_axis_D_vel_max)begin
                                    s_vel_data_valid    <= 1'b1;
                                end else begin
                                    s_vel_data_valid    <= 1'b0;
                                end
                            end
                        'd2:
                            begin
                                if(i_act_pva_data[31]==1'b0 && i_act_pva_data<= s_axis_D_acc_max)begin
                                    s_acc_data_valid    <= 1'b1;
                                end else if(i_act_pva_data[31]==1'b1 && i_act_pva_data<= s_axis_D_acc_min)begin
                                    s_acc_data_valid    <= 1'b1;
                                end else begin
                                    s_acc_data_valid    <= 1'b0;
                                end
                            end
                        default: ;
                    endcase        
                end 
            'd4: //axis E
                begin
                    case(s_data_loop_cnt)
                        'd0:
                            begin
                                if(i_act_pva_data[31]==1'b0 && i_act_pva_data<= s_axis_E_pos_max)begin
                                    s_pos_data_valid    <= 1'b1;
                                end else begin
                                    s_pos_data_valid    <= 1'b0;
                                end
                            end
                        'd1:
                            begin
                                if(i_act_pva_data[31]==1'b0 && i_act_pva_data<= s_axis_E_vel_max)begin
                                    s_vel_data_valid    <= 1'b1;
                                end else begin
                                    s_vel_data_valid    <= 1'b0;
                                end
                            end
                        'd2:
                            begin
                                if(i_act_pva_data[31]==1'b0 && i_act_pva_data<= s_axis_E_acc_max)begin
                                    s_acc_data_valid    <= 1'b1;
                                end else if(i_act_pva_data[31]==1'b1 && i_act_pva_data<= s_axis_E_acc_min)begin
                                    s_acc_data_valid    <= 1'b1;
                                end else begin
                                    s_acc_data_valid    <= 1'b0;
                                end
                            end
                        default: ;
                    endcase        
                end
            'd5: //axis F
                begin
                    case(s_data_loop_cnt)
                        'd0:
                            begin
                                if(i_act_pva_data[31]==1'b0 && i_act_pva_data<= s_axis_F_pos_max)begin
                                    s_pos_data_valid    <= 1'b1;
                                end else begin
                                    s_pos_data_valid    <= 1'b0;
                                end
                            end
                        'd1:
                            begin
                                if(i_act_pva_data[31]==1'b0 && i_act_pva_data<= s_axis_F_vel_max)begin
                                    s_vel_data_valid    <= 1'b1;
                                end else begin
                                    s_vel_data_valid    <= 1'b0;
                                end
                            end
                        'd2:
                            begin
                                if(i_act_pva_data[31]==1'b0 && i_act_pva_data<= s_axis_F_acc_max)begin
                                    s_acc_data_valid    <= 1'b1;
                                end else if(i_act_pva_data[31]==1'b1 && i_act_pva_data<= s_axis_F_acc_min)begin
                                    s_acc_data_valid    <= 1'b1;
                                end else begin
                                    s_acc_data_valid    <= 1'b0;
                                end
                            end
                        default: ;
                    endcase        
                end 
            default:
                begin
                    s_pos_data_valid    <= 1'b0;
                    s_vel_data_valid    <= 1'b0;
                    s_acc_data_valid    <= 1'b0;
                end
        endcase
    end else begin
        s_pos_data_valid    <= 1'b0;
        s_vel_data_valid    <= 1'b0;
        s_acc_data_valid    <= 1'b0;
    end
end

// when this frame received,judge the record,if it equal 18, this frame data is OK.else discard this frame data.
always@(posedge i_sys_clk or negedge i_rst_n)begin
    if(!i_rst_n)begin
        s_data_valid_record  <= 'd0;
        s_axis_A_pos_data    <= 'd0;
        s_axis_A_vel_data    <= 'd0;
        s_axis_A_acc_data    <= 'd0;
        s_axis_B_pos_data    <= 'd0;
        s_axis_B_vel_data    <= 'd0;
        s_axis_B_acc_data    <= 'd0;
        s_axis_C_pos_data    <= 'd0;
        s_axis_C_vel_data    <= 'd0;
        s_axis_C_acc_data    <= 'd0;
        s_axis_D_pos_data    <= 'd0;
        s_axis_D_vel_data    <= 'd0;
        s_axis_D_acc_data    <= 'd0;
        s_axis_E_pos_data    <= 'd0;
        s_axis_E_vel_data    <= 'd0;
        s_axis_E_acc_data    <= 'd0;
        s_axis_F_pos_data    <= 'd0;
        s_axis_F_vel_data    <= 'd0;
        s_axis_F_acc_data    <= 'd0;
    end else if(s_act_pva_data_en)begin
        case(s_axis_cnt_reg)
            'd0://axis A pva data stored
                begin
                    case(s_data_loop_cnt_reg)
                        'd0:
                            begin
                                if(s_pos_data_valid)begin
                                    s_axis_A_pos_data   <= s_act_pva_data;
                                    s_data_valid_record <= s_data_valid_record + 1'b1;
                                end else begin
                                    s_axis_A_pos_data   <= s_axis_A_pos_data;               //keep the former frame data; 
                                    s_data_valid_record <= s_data_valid_record;
                                end
                            end
                        'd1:
                            begin
                                if(s_vel_data_valid)begin
                                    s_axis_A_vel_data   <= s_act_pva_data;
                                    s_data_valid_record <= s_data_valid_record + 1'b1;
                                end else begin
                                    s_axis_A_vel_data   <= s_axis_A_vel_data;               //keep the former frame data; 
                                    s_data_valid_record <= s_data_valid_record;
                                end
                            end
                        'd2:
                            begin
                                if(s_acc_data_valid)begin
                                    s_axis_A_acc_data   <= s_act_pva_data;
                                    s_data_valid_record <= s_data_valid_record + 1'b1;
                                end else begin
                                    s_axis_A_acc_data   <= s_axis_A_acc_data;               //keep the former frame data; 
                                    s_data_valid_record <= s_data_valid_record;
                                end
                            end
                        default: ;
                    endcase
                end
            'd1://axis B pva data stored
                begin
                    case(s_data_loop_cnt_reg)
                        'd0:
                            begin
                                if(s_pos_data_valid)begin
                                    s_axis_B_pos_data   <= s_act_pva_data;
                                    s_data_valid_record <= s_data_valid_record + 1'b1;
                                end else begin
                                    s_axis_B_pos_data   <= s_axis_B_pos_data;               //keep the former frame data; 
                                    s_data_valid_record <= s_data_valid_record;
                                end
                            end
                        'd1:
                            begin
                                if(s_vel_data_valid)begin
                                    s_axis_B_vel_data   <= s_act_pva_data;
                                    s_data_valid_record <= s_data_valid_record + 1'b1;
                                end else begin
                                    s_axis_B_vel_data   <= s_axis_B_vel_data;               //keep the former frame data; 
                                    s_data_valid_record <= s_data_valid_record;
                                end
                            end
                        'd2:
                            begin
                                if(s_acc_data_valid)begin
                                    s_axis_B_acc_data   <= s_act_pva_data;
                                    s_data_valid_record <= s_data_valid_record + 1'b1;
                                end else begin
                                    s_axis_B_acc_data   <= s_axis_B_acc_data;               //keep the former frame data; 
                                    s_data_valid_record <= s_data_valid_record;
                                end
                            end
                        default: ;
                    endcase
                end
            'd2://axis C pva data stored
                begin
                    case(s_data_loop_cnt_reg)
                        'd0:
                            begin
                                if(s_pos_data_valid)begin
                                    s_axis_C_pos_data   <= s_act_pva_data;
                                    s_data_valid_record <= s_data_valid_record + 1'b1;
                                end else begin
                                    s_axis_C_pos_data   <= s_axis_C_pos_data;               //keep the former frame data; 
                                    s_data_valid_record <= s_data_valid_record;
                                end
                            end
                        'd1:
                            begin
                                if(s_vel_data_valid)begin
                                    s_axis_C_vel_data   <= s_act_pva_data;
                                    s_data_valid_record <= s_data_valid_record + 1'b1;
                                end else begin
                                    s_axis_C_vel_data   <= s_axis_C_vel_data;               //keep the former frame data; 
                                    s_data_valid_record <= s_data_valid_record;
                                end
                            end
                        'd2:
                            begin
                                if(s_acc_data_valid)begin
                                    s_axis_C_acc_data   <= s_act_pva_data;
                                    s_data_valid_record <= s_data_valid_record + 1'b1;
                                end else begin
                                    s_axis_C_acc_data   <= s_axis_C_acc_data;               //keep the former frame data; 
                                    s_data_valid_record <= s_data_valid_record;
                                end
                            end
                        default: ;
                    endcase
                end
            'd3://axis D pva data stored
                begin
                    case(s_data_loop_cnt_reg)
                        'd0:
                            begin
                                if(s_pos_data_valid)begin
                                    s_axis_D_pos_data   <= s_act_pva_data;
                                    s_data_valid_record <= s_data_valid_record + 1'b1;
                                end else begin
                                    s_axis_D_pos_data   <= s_axis_D_pos_data;               //keep the former frame data; 
                                    s_data_valid_record <= s_data_valid_record;
                                end
                            end
                        'd1:
                            begin
                                if(s_vel_data_valid)begin
                                    s_axis_D_vel_data   <= s_act_pva_data;
                                    s_data_valid_record <= s_data_valid_record + 1'b1;
                                end else begin
                                    s_axis_D_vel_data   <= s_axis_D_vel_data;               //keep the former frame data; 
                                    s_data_valid_record <= s_data_valid_record;
                                end
                            end
                        'd2:
                            begin
                                if(s_acc_data_valid)begin
                                    s_axis_D_acc_data   <= s_act_pva_data;
                                    s_data_valid_record <= s_data_valid_record + 1'b1;
                                end else begin
                                    s_axis_D_acc_data   <= s_axis_D_acc_data;               //keep the former frame data; 
                                    s_data_valid_record <= s_data_valid_record;
                                end
                            end
                        default: ;
                    endcase
                end
            'd4://axis E pva data stored
                begin
                    case(s_data_loop_cnt_reg)
                        'd0:
                            begin
                                if(s_pos_data_valid)begin
                                    s_axis_E_pos_data   <= s_act_pva_data;
                                    s_data_valid_record <= s_data_valid_record + 1'b1;
                                end else begin
                                    s_axis_E_pos_data   <= s_axis_E_pos_data;               //keep the former frame data; 
                                    s_data_valid_record <= s_data_valid_record;
                                end
                            end
                        'd1:
                            begin
                                if(s_vel_data_valid)begin
                                    s_axis_E_vel_data   <= s_act_pva_data;
                                    s_data_valid_record <= s_data_valid_record + 1'b1;
                                end else begin
                                    s_axis_E_vel_data   <= s_axis_E_vel_data;               //keep the former frame data; 
                                    s_data_valid_record <= s_data_valid_record;
                                end
                            end
                        'd2:
                            begin
                                if(s_acc_data_valid)begin
                                    s_axis_E_acc_data   <= s_act_pva_data;
                                    s_data_valid_record <= s_data_valid_record + 1'b1;
                                end else begin
                                    s_axis_E_acc_data   <= s_axis_E_acc_data;               //keep the former frame data; 
                                    s_data_valid_record <= s_data_valid_record;
                                end
                            end
                        default: ;
                    endcase
                end
            'd5://axis F pva data stored
                begin
                    case(s_data_loop_cnt_reg)
                        'd0:
                            begin
                                if(s_pos_data_valid)begin
                                    s_axis_F_pos_data   <= s_act_pva_data;
                                    s_data_valid_record <= s_data_valid_record + 1'b1;
                                end else begin
                                    s_axis_F_pos_data   <= s_axis_F_pos_data;               //keep the former frame data; 
                                    s_data_valid_record <= s_data_valid_record;
                                end
                            end
                        'd1:
                            begin
                                if(s_vel_data_valid)begin
                                    s_axis_F_vel_data   <= s_act_pva_data;
                                    s_data_valid_record <= s_data_valid_record + 1'b1;
                                end else begin
                                    s_axis_F_vel_data   <= s_axis_F_vel_data;               //keep the former frame data; 
                                    s_data_valid_record <= s_data_valid_record;
                                end
                            end
                        'd2:
                            begin
                                if(s_acc_data_valid)begin
                                    s_axis_F_acc_data   <= s_act_pva_data;
                                    s_data_valid_record <= s_data_valid_record + 1'b1;
                                end else begin
                                    s_axis_F_acc_data   <= s_axis_F_acc_data;               //keep the former frame data; 
                                    s_data_valid_record <= s_data_valid_record;
                                end
                            end
                        default: ;
                    endcase
                end
            default:
                begin
                    s_data_valid_record  <= 'd0;
                    s_axis_A_pos_data    <= s_axis_A_pos_data;
                    s_axis_A_vel_data    <= s_axis_A_vel_data;
                    s_axis_A_acc_data    <= s_axis_A_acc_data;
                    s_axis_B_pos_data    <= s_axis_B_pos_data;
                    s_axis_B_vel_data    <= s_axis_B_vel_data;
                    s_axis_B_acc_data    <= s_axis_B_acc_data;
                    s_axis_C_pos_data    <= s_axis_C_pos_data;
                    s_axis_C_vel_data    <= s_axis_C_vel_data;
                    s_axis_C_acc_data    <= s_axis_C_acc_data;
                    s_axis_D_pos_data    <= s_axis_D_pos_data;
                    s_axis_D_vel_data    <= s_axis_D_vel_data;
                    s_axis_D_acc_data    <= s_axis_D_acc_data;
                    s_axis_E_pos_data    <= s_axis_E_pos_data;
                    s_axis_E_vel_data    <= s_axis_E_vel_data;
                    s_axis_E_acc_data    <= s_axis_E_acc_data;
                    s_axis_F_pos_data    <= s_axis_F_pos_data;
                    s_axis_F_vel_data    <= s_axis_F_vel_data;
                    s_axis_F_acc_data    <= s_axis_F_acc_data;
                end
        endcase
    end else begin
        s_data_valid_record  <= 'd0;
        s_axis_A_pos_data    <= s_axis_A_pos_data;
        s_axis_A_vel_data    <= s_axis_A_vel_data;
        s_axis_A_acc_data    <= s_axis_A_acc_data;
        s_axis_B_pos_data    <= s_axis_B_pos_data;
        s_axis_B_vel_data    <= s_axis_B_vel_data;
        s_axis_B_acc_data    <= s_axis_B_acc_data;
        s_axis_C_pos_data    <= s_axis_C_pos_data;
        s_axis_C_vel_data    <= s_axis_C_vel_data;
        s_axis_C_acc_data    <= s_axis_C_acc_data;
        s_axis_D_pos_data    <= s_axis_D_pos_data;
        s_axis_D_vel_data    <= s_axis_D_vel_data;
        s_axis_D_acc_data    <= s_axis_D_acc_data;
        s_axis_E_pos_data    <= s_axis_E_pos_data;
        s_axis_E_vel_data    <= s_axis_E_vel_data;
        s_axis_E_acc_data    <= s_axis_E_acc_data;
        s_axis_F_pos_data    <= s_axis_F_pos_data;
        s_axis_F_vel_data    <= s_axis_F_vel_data;
        s_axis_F_acc_data    <= s_axis_F_acc_data;
    end
end
always@(posedge i_sys_clk or negedge i_rst_n)begin
    if(!i_rst_n)begin
        o_act_pva_axis_data_en  <= 1'b0;
    end else if(s_axis_cnt_reg=='d5 && s_data_loop_cnt_reg == 'd3)begin
        o_act_pva_axis_data_en  <= 1'b1;
    end else begin
        o_act_pva_axis_data_en  <= 1'b0;
    end
end


assign o_act_pva_axis_A_pos_data = s_axis_A_pos_data;
assign o_act_pva_axis_A_vel_data = s_axis_A_vel_data;
assign o_act_pva_axis_A_acc_data = s_axis_A_acc_data;
assign o_act_pva_axis_B_pos_data = s_axis_B_pos_data;
assign o_act_pva_axis_B_vel_data = s_axis_B_vel_data;
assign o_act_pva_axis_B_acc_data = s_axis_B_acc_data;
assign o_act_pva_axis_C_pos_data = s_axis_C_pos_data;
assign o_act_pva_axis_C_vel_data = s_axis_C_vel_data;
assign o_act_pva_axis_C_acc_data = s_axis_C_acc_data;
assign o_act_pva_axis_D_pos_data = s_axis_D_pos_data;
assign o_act_pva_axis_D_vel_data = s_axis_D_vel_data;
assign o_act_pva_axis_D_acc_data = s_axis_D_acc_data;
assign o_act_pva_axis_E_pos_data = s_axis_E_pos_data;
assign o_act_pva_axis_E_vel_data = s_axis_E_vel_data;
assign o_act_pva_axis_E_acc_data = s_axis_E_acc_data;
assign o_act_pva_axis_F_pos_data = s_axis_F_pos_data;
assign o_act_pva_axis_F_vel_data = s_axis_F_vel_data;
assign o_act_pva_axis_F_acc_data = s_axis_F_acc_data;

endmodule