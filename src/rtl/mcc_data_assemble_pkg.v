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
// 4            s_pkg_length         uint_32        byte 
// 4            pkg_cnt              uint_32        num
// 4            reserved             uint_32        num
// 4            Message_ID           uint_32        num
//
//Message_ID:
//011:request ack     4byte
//012:request nack    4byte
//200:status data     variable
//////////////////////////////////////////////////////////////////////////////////
module mcc_data_assemble_pkg
    (
        input               i_sys_clk,
        input               i_rst_n,
        input               i_req_ack_start_en,
        input               i_req_nack_start_en,
        input               i_status_request_start_en,
        input [2:0]         i_status_request_data,
        
        output              o_assemble_pkg_done,
        output              o_assemble_pkg_is_running,
		output [15:0]       o_pkg_data_length,
		output [15:0]       o_pkg_data_total_length,
        output              o_pkg_ram_wr_en,
        output[8:0]         o_pkg_ram_wr_addr,
        output[31:0]        o_pkg_ram_wr_data,
        output  reg         o_pkg_send_udp_req

    );

`ifdef SIMULATION
parameter          Idle                             = "Idle" ;
parameter          Req_Ack_Pkg                      = "Req_Ack_Pkg" ;
parameter          Req_Nack_Pkg                     = "Req_Nack_Pkg" ;
parameter          Status_Data_Pkg                  = "Status_Data_Pkg" ;
parameter          Message_Header                   = "Message_Header" ;
parameter          Ack_Nack_Data                    = "Ack_Nack_Data" ;
parameter          Status_Response_Section_0        = "Status_Response_Section_0" ;
parameter          Write_Pkg_Done                   = "Write_Pkg_Done" ;
parameter          state_wid_msb                    = 200;
`else    
parameter          Idle                             = 4'd0 ;
parameter          Req_Ack_Pkg                      = 4'd1 ;
parameter          Req_Nack_Pkg                     = 4'd2 ;
parameter          Status_Data_Pkg                  = 4'd3 ;
parameter          Message_Header                   = 4'd4 ;
parameter          Ack_Nack_Data                    = 4'd5 ;
parameter          Status_Response_Section_0        = 4'd6 ;
parameter          Status_Response_Section_1        = 4'd7 ;
parameter          Write_Pkg_Done                   = 4'd8 ;
parameter          state_wid_msb                    = 4;
`endif

parameter           REQ_ACK                         =  2'd0;
parameter           REQ_NACK                        =  2'd1; 
parameter           STATUS                          =  2'd2;

parameter            None                            = 3'd0;
parameter            DOF_Option_1                    = 3'd1;
parameter            Length_Option_2                 = 3'd2;
parameter            Actuator_Data_Option_3          = 3'd3;
parameter            Accelerometer_Option_4          = 3'd4;
parameter            Alarm_List_Option_5             = 3'd5;
parameter            Dof_Extended_Data_Option_6      = 3'd6;
parameter            Automated_Test_Status_Option_7  = 3'd7;


reg [state_wid_msb-1:0]           s_fsm_cur;
reg [state_wid_msb-1:0]           s_fsm_next;
reg[31:0]           s_pkg_seq_cnt;
reg[31:0]           s_pkg_length;
reg[31:0]           s_status_data_pkg_length;
reg[31:0]           s_pkg_message_ID;
reg                 s_pkg_ram_wr_en;
reg[8:0]            s_pkg_ram_wr_addr;
reg[31:0]           s_pkg_ram_wr_data;

reg[31:0]           s_machine_status_word='d255;
reg[31:0]           s_discrete_io_word0=10;
reg[31:0]           s_discrete_io_word1=11;
reg[31:0]           s_optional_status_data=2;
reg[8:0]            s_pkg_cnt;
reg                 s_pkg_cnt_done;
reg  [1:0]          s_message_ID_type;
reg                 s_req_ack_wait;
reg                 s_req_nack_wait;
reg                 s_status_req_ack_wait;

always@(posedge i_sys_clk or negedge i_rst_n)
begin
    if(!i_rst_n)begin
        s_req_ack_wait          <= 1'b0;    
        s_req_nack_wait         <= 1'b0;
        s_status_req_ack_wait   <= 1'b0;
    end else if(i_req_ack_start_en)begin
        s_req_ack_wait          <= 1'b1;
    end else if(i_req_nack_start_en)begin
        s_req_nack_wait         <= 1'b1;
    end else if(i_status_request_start_en)begin
        s_status_req_ack_wait   <= 1'b1;
    end else if(s_fsm_cur == Idle)begin
        s_req_ack_wait          <= 1'b0;
        if(s_req_ack_wait)begin
            s_req_nack_wait         <= s_req_nack_wait;
        end else begin
            s_req_nack_wait         <= 1'b0;
        end
        if(s_req_nack_wait|s_req_ack_wait)begin
            s_status_req_ack_wait   <= s_status_req_ack_wait;
        end else begin
            s_status_req_ack_wait   <= 1'b0;
        end
    end else begin
        s_req_ack_wait          <= s_req_ack_wait;    
        s_req_nack_wait         <= s_req_nack_wait;
        s_status_req_ack_wait   <= s_status_req_ack_wait;
    end
end


always@(posedge i_sys_clk or negedge i_rst_n)
begin
    if(!i_rst_n)begin
        s_optional_status_data <= 'd0;
        s_status_data_pkg_length <= 'd0;
    end else if(i_status_request_start_en)begin
        case(i_status_request_data)
            None:  
                begin
                    s_optional_status_data <= 'd0;
                    s_status_data_pkg_length <= 'd0;
                end
            DOF_Option_1:
                begin
                    s_optional_status_data <= 'd1;
                    s_status_data_pkg_length <= 'd13;
                end
            Length_Option_2:
                begin
                    s_optional_status_data <= 'd2; 
                    s_status_data_pkg_length <= 'd13;  
                end
            Actuator_Data_Option_3:
                begin 
                    s_optional_status_data <= 'd3; 
                    s_status_data_pkg_length <= 'd68;
                end
            Accelerometer_Option_4:
                begin
                    s_optional_status_data <= 'd4;
                    s_status_data_pkg_length <= 'd10;
                end
            Alarm_List_Option_5:
                begin
                    s_optional_status_data <= 'd5;
                    s_status_data_pkg_length <= 'd40;
                end
            Dof_Extended_Data_Option_6:
                begin
                    s_optional_status_data <= 'd6;
                    s_status_data_pkg_length <= 'd43;
                end
            Automated_Test_Status_Option_7:
                begin
                    s_optional_status_data <= 'd7;
                    s_status_data_pkg_length <= 'd6;
                end
            default:
                begin
                    s_optional_status_data <= 'd0;
                    s_status_data_pkg_length <= 'd0;
                end
        endcase
    end else begin
        s_optional_status_data <= s_optional_status_data;
        s_status_data_pkg_length <= s_status_data_pkg_length;
    end


end

always@(posedge i_sys_clk or negedge i_rst_n)
begin
    if(!i_rst_n)begin
        s_message_ID_type   <= REQ_NACK;
    end else if(i_req_ack_start_en)begin
        s_message_ID_type   <= REQ_ACK;
    end else if(i_req_nack_start_en)begin
        s_message_ID_type   <= REQ_NACK;
    end else if(i_status_request_start_en)begin
        s_message_ID_type   <= STATUS;
    end else begin
        s_message_ID_type   <= s_message_ID_type;
    end
end

always@(posedge i_sys_clk or negedge i_rst_n)begin
    if(!i_rst_n)begin
        s_fsm_cur   <= Idle;
    end else begin
        s_fsm_cur   <= s_fsm_next;
    end
end                            
  
always@(*)
begin
    case(s_fsm_cur)
        Idle:
            begin
                if(s_req_ack_wait)begin
                    s_fsm_next <= Req_Ack_Pkg;
                end else if(s_req_nack_wait)begin
                    s_fsm_next <= Req_Nack_Pkg;
                end else if(s_status_req_ack_wait)begin
                    s_fsm_next <= Status_Data_Pkg;
                end else begin
                    s_fsm_next <= Idle;
                end
            end
        Req_Ack_Pkg:
            begin
                s_fsm_next <= Message_Header;
            end
        Req_Nack_Pkg:
            begin
                s_fsm_next <= Message_Header;
            end
        Status_Data_Pkg:
            begin
                s_fsm_next <= Message_Header;
            end  
        Message_Header:
            begin
                if(s_pkg_cnt_done)begin
                    if(s_message_ID_type == REQ_ACK || s_message_ID_type == REQ_NACK)begin
                        s_fsm_next <=   Ack_Nack_Data;
                    end else begin
                        s_fsm_next <=   Status_Response_Section_0;
                    end                    
                end else begin
                    s_fsm_next <= Message_Header;
                end
            end
        Ack_Nack_Data:
            begin
                s_fsm_next <=   Write_Pkg_Done;
            end
        Status_Response_Section_0:
            begin
                if(s_pkg_cnt_done)begin
                    s_fsm_next <= Status_Response_Section_1;
                end else begin
                    s_fsm_next <=   Status_Response_Section_0;
                end
            end
        Status_Response_Section_1:
            begin
            
            end
        Write_Pkg_Done:
            begin
                s_fsm_next <= Idle;
            end
        default:;
    endcase
end


always@(posedge i_sys_clk or negedge i_rst_n)begin
    if(!i_rst_n)begin
        s_pkg_length  <= 'd0;//header not included
    end else if(s_fsm_next == Req_Ack_Pkg || s_fsm_next == Req_Nack_Pkg)begin
        s_pkg_length  <= 'd1;
    end else if(s_fsm_next == Status_Data_Pkg)begin
        s_pkg_length  <= s_status_data_pkg_length;
    end else begin
        s_pkg_length  <= s_pkg_length;
    end
end

always@(posedge i_sys_clk or negedge i_rst_n)begin
    if(!i_rst_n)begin
        s_pkg_message_ID  <= 'd0;
    end else if(s_fsm_next == Req_Ack_Pkg)begin
        s_pkg_message_ID  <= 'd11;
    end else if(s_fsm_next == Req_Nack_Pkg)begin
        s_pkg_message_ID  <= 'd12;
    end else if(s_fsm_next == Status_Data_Pkg)begin
        s_pkg_message_ID  <= 'd200;
    end else begin
        s_pkg_message_ID  <= s_pkg_message_ID;
    end
end

always@(posedge i_sys_clk or negedge i_rst_n)begin
    if(!i_rst_n)begin
        s_pkg_seq_cnt     <= 'd0;
    end else if(s_fsm_next == Req_Ack_Pkg || s_fsm_next == Req_Nack_Pkg || s_fsm_next == Status_Data_Pkg)begin
        s_pkg_seq_cnt     <= s_pkg_seq_cnt + 1'b1;
    end else begin
        s_pkg_seq_cnt     <= s_pkg_seq_cnt;
    end
end

always@(posedge i_sys_clk or negedge i_rst_n)begin
    if(!i_rst_n)begin
        s_pkg_cnt           <= 'd0;
        s_pkg_cnt_done      <= 1'b0;
        s_pkg_ram_wr_en     <= 1'b0;
    end else if(s_fsm_next == Message_Header || s_fsm_next == Ack_Nack_Data)begin
        if(s_pkg_cnt=='d3)begin
            s_pkg_cnt       <= s_pkg_cnt + 1'b1;
            s_pkg_cnt_done  <= 1'b1;
        end else begin
            s_pkg_cnt       <= s_pkg_cnt + 1'b1;
            s_pkg_cnt_done  <= 1'b0;
        end
        s_pkg_ram_wr_en     <= 1'b1;
    end else if(s_fsm_next == Status_Response_Section_0)begin
        if(s_pkg_cnt=='d9)begin
            s_pkg_cnt       <= 0;
            s_pkg_cnt_done  <= 1'b1;
        end else begin
            s_pkg_cnt       <= s_pkg_cnt + 1'b1;
            s_pkg_cnt_done  <= 1'b0;
        end
        s_pkg_ram_wr_en     <= 1'b1;
    end else begin
        s_pkg_cnt           <= 'd0;
        s_pkg_cnt_done      <= 1'b0;
        s_pkg_ram_wr_en     <= 1'b0;
    end
end

always@(posedge i_sys_clk or negedge i_rst_n)begin
    if(!i_rst_n)begin
        o_pkg_send_udp_req <= 1'b0;
    end else if(s_fsm_next == Write_Pkg_Done)begin
        o_pkg_send_udp_req <= 1'b1;
    end else begin
        o_pkg_send_udp_req <= 1'b0;
    end
end

always@(posedge i_sys_clk or negedge i_rst_n)begin
    if(!i_rst_n)begin
        s_pkg_ram_wr_addr <= 'd0;
    end else begin
        s_pkg_ram_wr_addr <= s_pkg_cnt;
    end
end


always@(posedge i_sys_clk or negedge i_rst_n)begin
    if(!i_rst_n)begin
        s_pkg_ram_wr_data   <= 'd0;
    end else begin
        case(s_pkg_cnt)
            'd0:    s_pkg_ram_wr_data <= s_pkg_length<<2;
            'd1:    s_pkg_ram_wr_data <= s_pkg_seq_cnt;
            'd2:    s_pkg_ram_wr_data <= 'd0;
            'd3:    s_pkg_ram_wr_data <= s_pkg_message_ID;
            'd4:    
                begin
                    if(s_fsm_next == Ack_Nack_Data)begin
                        s_pkg_ram_wr_data <= 'd0;
                    end else begin
                        s_pkg_ram_wr_data <= s_machine_status_word;
                    end
                end
            'd5:    s_pkg_ram_wr_data <= s_discrete_io_word0;
            'd6:    s_pkg_ram_wr_data <= s_discrete_io_word1;  
            'd9:    s_pkg_ram_wr_data <= s_optional_status_data;  
            default:s_pkg_ram_wr_data <= 'd0;
        endcase
    end
end
assign o_assemble_pkg_is_running  =  (s_fsm_next ==Idle)?0:1;               
assign o_assemble_pkg_done        =  (s_fsm_next ==Write_Pkg_Done)?1:0;        
assign o_pkg_data_length          =  (s_pkg_length<<2)+16+8; //pkg_length*4 + message header + udp header    //add udp header length(data pkg length  + udp header 8bytes)
assign o_pkg_data_total_length    =  o_pkg_data_length + 20;                      //add ip header length
assign o_pkg_ram_wr_en            =  s_pkg_ram_wr_en;
assign o_pkg_ram_wr_addr          =  s_pkg_ram_wr_addr;
assign o_pkg_ram_wr_data          =  s_pkg_ram_wr_data;
endmodule