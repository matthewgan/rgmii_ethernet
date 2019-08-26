`timescale 1ns / 1ps  
//////////////////////////////////////////////////////////////////////////////////
// Module Name:    ethernet_test 
//////////////////////////////////////////////////////////////////////////////////
module ethernet_test(
   input sys_clk,      // ¿ª·¢°åÉÏ²î·ÖÊäÈëÊ±ÖÓ200Mhz
   input key1,

	output e_reset,
	output e_mdc,
	inout  e_mdio,

	output[3:0] rgmii_txd,
	output rgmii_txctl,
	output rgmii_txc,
	input[3:0] rgmii_rxd,
	input rgmii_rxctl,
	input rgmii_rxc
  
    );
parameter INS_WIDTH = 32;
wire reset_n;
wire   [ 7:0]   gmii_txd;
wire            gmii_tx_en;
wire            gmii_tx_er;
wire            gmii_tx_clk;
wire            gmii_crs;
wire            gmii_col;
wire   [ 7:0]   gmii_rxd;
wire            gmii_rx_dv;
wire            gmii_rx_er;
wire            gmii_rx_clk;
wire  [ 1:0]    speed_selection; // 1x gigabit, 01 100Mbps, 00 10mbps
wire            duplex_mode;     // 1 full, 0 half

assign speed_selection = 2'b10;
assign duplex_mode = 1'b1;
assign e_reset = reset_n;


util_gmii_to_rgmii util_gmii_to_rgmii_m0(
	.reset(1'b0),
	
	.rgmii_td(rgmii_txd),
	.rgmii_tx_ctl(rgmii_txctl),
	.rgmii_txc(rgmii_txc),
	.rgmii_rd(rgmii_rxd),
	.rgmii_rx_ctl(rgmii_rxctl),
	.rgmii_rxc(rgmii_rxc),
	
	.gmii_txd(gmii_txd),
	.gmii_tx_en(gmii_tx_en),
	.gmii_tx_er(gmii_tx_er),
	.gmii_tx_clk(gmii_tx_clk),
	.gmii_crs(gmii_crs),
	.gmii_col(gmii_col),
	.gmii_rxd(gmii_rxd),
	.gmii_rx_dv(gmii_rx_dv),
	.gmii_rx_er(gmii_rx_er),
	.gmii_rx_clk(gmii_rx_clk),
	.speed_selection(speed_selection),
	.duplex_mode(duplex_mode));
//////////////////////////////////////////////////////////

wire [31:0] ram_wr_data;
wire [31:0] ram_rd_data;
wire [8:0] ram_wr_addr;
wire [8:0] ram_rd_addr;
reg ram_wren_i;
wire [31:0] datain_reg;

wire [3:0] tx_state;
wire [3:0] rx_state;
wire [15:0] rx_total_length;    
wire [15:0] tx_total_length;    
wire [15:0] rx_data_length;     
wire [15:0] tx_data_length;     
wire o_udp_data_receive_done;
reg ram_wr_finish;
reg [31:0] udp_data [4:0];                        
reg [8:0] ram_addr_i;
reg [31:0] ram_data_i;
reg [4:0] i;
wire data_o_valid;
wire wea;
wire [8:0] addra;
wire [31:0] dina;
wire        s_udp_data_end;
wire [8:0] ram_rd_ins_addr;
wire [31:0] ram_rd_ins_data;
wire [8:0] ram_rd_ins_addr_reg;

wire                  o_dof_data_en;  
wire [INS_WIDTH-1:0]  o_dof_data;            
wire                  o_act_pos_data_en;               
wire [INS_WIDTH-1:0]  o_act_pos_data;                   
wire                  o_mot_cue_data_en;                
wire [INS_WIDTH-1:0]  o_mot_cue_data;                   
wire                  o_playback_data_en;               
wire [INS_WIDTH-1:0]  o_playback_data;                  
wire                  o_ext_mot_cue_data_en;           
wire [INS_WIDTH-1:0]  o_ext_mot_cue_data;               
wire                  o_act_pva_data_en;                
wire [INS_WIDTH-1:0]  o_act_pva_data;
wire [INS_WIDTH-1:0]  o_act_pva_axis_A_pos_data;
wire [INS_WIDTH-1:0]  o_act_pva_axis_A_vel_data;
wire [INS_WIDTH-1:0]  o_act_pva_axis_A_acc_data;
wire [INS_WIDTH-1:0]  o_act_pva_axis_B_pos_data;
wire [INS_WIDTH-1:0]  o_act_pva_axis_B_vel_data;
wire [INS_WIDTH-1:0]  o_act_pva_axis_B_acc_data;
wire [INS_WIDTH-1:0]  o_act_pva_axis_C_pos_data;
wire [INS_WIDTH-1:0]  o_act_pva_axis_C_vel_data;
wire [INS_WIDTH-1:0]  o_act_pva_axis_C_acc_data;
wire [INS_WIDTH-1:0]  o_act_pva_axis_D_pos_data;
wire [INS_WIDTH-1:0]  o_act_pva_axis_D_vel_data;
wire [INS_WIDTH-1:0]  o_act_pva_axis_D_acc_data;
wire [INS_WIDTH-1:0]  o_act_pva_axis_E_pos_data;
wire [INS_WIDTH-1:0]  o_act_pva_axis_E_vel_data;
wire [INS_WIDTH-1:0]  o_act_pva_axis_E_acc_data;
wire [INS_WIDTH-1:0]  o_act_pva_axis_F_pos_data;
wire [INS_WIDTH-1:0]  o_act_pva_axis_F_vel_data;
wire [INS_WIDTH-1:0]  o_act_pva_axis_F_acc_data;
wire                  o_act_pva_axis_data_en   ;

wire        s_pkg_ram_wr_en;
wire[8:0]   s_pkg_ram_wr_addr;
wire[31:0]  s_pkg_ram_wr_data;
wire        s_pkg_send_udp_req;
wire        s_req_ack_start_en;
wire        s_req_nack_start_en;
wire        s_status_request_start_en;
wire[2:0]   s_status_request_data;
wire[15:0]      s_pkg_data_length;
wire[15:0]      s_pkg_data_total_length;
//assign wea = ram_wr_finish ? data_o_valid : ram_wren_i;
//assign addra = ram_wr_finish ? ram_wr_addr : ram_addr_i;
//assign dina = ram_wr_finish ? ram_wr_data : ram_data_i;
//assign wea = data_o_valid;
//assign addra = ram_wr_addr;
//assign dina = ram_wr_data;
//assign wea = ram_wr_finish ? s_pkg_ram_wr_en : ram_wren_i;
//assign addra = ram_wr_finish ? s_pkg_ram_wr_addr : ram_addr_i;
//assign dina = ram_wr_finish ? s_pkg_ram_wr_data : ram_data_i;
//assign tx_data_length = o_udp_data_receive_done ? s_pkg_data_length : 16'd28;
//assign tx_total_length = o_udp_data_receive_done ? s_pkg_data_total_length : 16'd48;

assign wea = s_pkg_ram_wr_en;
assign addra = s_pkg_ram_wr_addr;
assign dina = s_pkg_ram_wr_data;
assign tx_data_length = s_pkg_data_length;
assign tx_total_length = s_pkg_data_total_length;
reset reset_m0(
	.clk(sys_clk),
	.key1(key1),
	.rst_n(reset_n)
);

//MDIO¼Ä´æÆ÷ÅäÖÃ
wire [15:0] phy_reg;
wire phy_init;           //PHY link up³õÊ¼»¯Íê³É
phy_reg_config phy_reg_config_inst( 
 	 .clock_50m(sys_clk),
 	 .reset_n(reset_n),
	 .phy_mdc(e_mdc),
	 .phy_mdio(e_mdio),
	 .phy_reg(phy_reg),
	 .phy_rst_n(),
	 .phy_init(phy_init)
					);	
//MDIO¼Ä´æÆ÷ÅäÖÃ
udp u1(
	.reset_n(reset_n),
	.g_clk(gmii_tx_clk),
	
	.e_rxc(gmii_rx_clk),
	.e_rxd(gmii_rxd),
	.e_rxdv(gmii_rx_dv),
	.e_txen(gmii_tx_en),
	.e_txd(gmii_txd),
	.e_txer(gmii_tx_er),		
	
	.data_o_valid(data_o_valid),          
	.ram_wr_data(ram_wr_data),            
	.rx_total_length(rx_total_length),    
	.rx_state(rx_state),                  
	.rx_data_length(rx_data_length),      
	.ram_wr_addr(ram_wr_addr),            
	.o_udp_data_receive_done(o_udp_data_receive_done),
    .o_udp_data_end(s_udp_data_end),
	.ram_rd_data(ram_rd_data),            
	.tx_state(tx_state),                  
	
	.tx_data_length(tx_data_length),      
	.tx_total_length(tx_total_length),    
	.ram_rd_addr(ram_rd_addr),             
    .i_pkg_send_udp_req(s_pkg_send_udp_req)
	);


ram ram_inst (       //tx ram
	.clka(gmii_rx_clk),                                  // input clka
	.wea(wea),                                     // input [0 : 0] wea
	.addra(addra),                                 // input [8 : 0] addra
	.dina(dina),                                   // input [31 : 0] dina
	
	.clkb(gmii_tx_clk),                                  // input clkb
	.addrb(ram_rd_addr),                           // input [8 : 0] addrb
	.doutb(ram_rd_data)                            // output [31 : 0] doutb
);
ila_0 ram_probe (
	.clk(gmii_rx_clk), // input wire clk


	.probe0({gmii_txd,gmii_tx_en,s_req_ack_start_en,s_pkg_send_udp_req,s_pkg_ram_wr_data,s_pkg_ram_wr_addr,s_pkg_ram_wr_en}) // input wire [59:0] probe0
);
blk_mem_gen_0 udp_receive_data (  //rx ram
  .clka(gmii_rx_clk),    // input wire clka
  .wea(data_o_valid),      // input wire [0 : 0] wea
  .addra(ram_wr_addr),  // input wire [8 : 0] addra
  .dina(ram_wr_data),    // input wire [31 : 0] dina
  
  .clkb(gmii_rx_clk),    // input wire clkb
  .addrb(ram_rd_ins_addr_reg),  // input wire [8 : 0] addrb
  .doutb(ram_rd_ins_data)  // output wire [31 : 0] doutb
);

mcc_data_assemble_pkg u_mcc_data_assemble_pkg
    (
        .i_sys_clk(gmii_rx_clk),
        .i_rst_n(reset_n),
        .i_req_ack_start_en(s_req_ack_start_en),
        .i_req_nack_start_en(s_req_nack_start_en),
        .i_status_request_start_en(s_status_request_start_en),
        .i_status_request_data(s_status_request_data),
        .o_assemble_pkg_done(),
        .o_assemble_pkg_is_running(),
		.o_pkg_data_length(s_pkg_data_length),
		.o_pkg_data_total_length(s_pkg_data_total_length),
        .o_pkg_ram_wr_en(s_pkg_ram_wr_en),
        .o_pkg_ram_wr_addr(s_pkg_ram_wr_addr),
        .o_pkg_ram_wr_data(s_pkg_ram_wr_data),
        .o_pkg_send_udp_req(s_pkg_send_udp_req)

    );
    
assign ram_rd_ins_addr_reg = ram_rd_ins_addr;
instruct_parsing#(.INS_WIDTH(32))
        u_instruct_parsing(
            .i_sys_clk(gmii_rx_clk),
            .i_rst_n(reset_n),

            .i_udp_data_receive_done(s_udp_data_end),
            .i_instruct_data(ram_rd_ins_data),
            .o_udp_data_rd_addr(ram_rd_ins_addr),
            .o_dof_data_en(o_dof_data_en),  
            .o_dof_data(o_dof_data),            
            .o_act_pos_data_en(o_act_pos_data_en),                
            .o_act_pos_data(o_act_pos_data),                   
            .o_mot_cue_data_en(o_mot_cue_data_en),                
            .o_mot_cue_data(o_mot_cue_data),                   
            .o_playback_data_en(o_playback_data_en),               
            .o_playback_data(o_playback_data),                  
            .o_ext_mot_cue_data_en(o_ext_mot_cue_data_en),            
            .o_ext_mot_cue_data(o_ext_mot_cue_data),               
            .o_act_pva_data_en(o_act_pva_data_en),                
            .o_act_pva_data(o_act_pva_data),
            .o_req_ack_start_en(s_req_ack_start_en),
            .o_req_nack_start_en(s_req_nack_start_en),
            .o_status_request_data(s_status_request_data),
            .o_status_request_start_en(s_status_request_start_en)

 
        );
actuator_pva #(.INS_WIDTH(32))
        u_actuator_pva(
            .i_sys_clk(gmii_rx_clk),
            .i_rst_n(reset_n),
            .i_act_pva_data_en(o_act_pva_data_en),                
            .i_act_pva_data(o_act_pva_data),    

            .o_act_pva_axis_A_pos_data(o_act_pva_axis_A_pos_data),
            .o_act_pva_axis_A_vel_data(o_act_pva_axis_A_vel_data),
            .o_act_pva_axis_A_acc_data(o_act_pva_axis_A_acc_data),
            .o_act_pva_axis_B_pos_data(o_act_pva_axis_B_pos_data),
            .o_act_pva_axis_B_vel_data(o_act_pva_axis_B_vel_data),
            .o_act_pva_axis_B_acc_data(o_act_pva_axis_B_acc_data),
            .o_act_pva_axis_C_pos_data(o_act_pva_axis_C_pos_data),
            .o_act_pva_axis_C_vel_data(o_act_pva_axis_C_vel_data),
            .o_act_pva_axis_C_acc_data(o_act_pva_axis_C_acc_data),
            .o_act_pva_axis_D_pos_data(o_act_pva_axis_D_pos_data),
            .o_act_pva_axis_D_vel_data(o_act_pva_axis_D_vel_data),
            .o_act_pva_axis_D_acc_data(o_act_pva_axis_D_acc_data),
            .o_act_pva_axis_E_pos_data(o_act_pva_axis_E_pos_data),
            .o_act_pva_axis_E_vel_data(o_act_pva_axis_E_vel_data),
            .o_act_pva_axis_E_acc_data(o_act_pva_axis_E_acc_data),
            .o_act_pva_axis_F_pos_data(o_act_pva_axis_F_pos_data),
            .o_act_pva_axis_F_vel_data(o_act_pva_axis_F_vel_data),
            .o_act_pva_axis_F_acc_data(o_act_pva_axis_F_acc_data),
            .o_act_pva_axis_data_en(o_act_pva_axis_data_en)
        );
/********************************************/
//´æ´¢´ý·¢ËÍµÄ×Ö·û
/********************************************/
always @(*)
begin     //¶¨Òå·¢ËÍµÄ×Ö·û
	 udp_data[0]<={"H","E","L","L"};   //´æ´¢×Ö·ûHELL 
	 udp_data[1]<={"O"," ","A","L"};   //´æ´¢×Ö·ûO¿Õ¸ñAL 
    udp_data[2]<={"I","N","X"," "};   //´æ´¢×Ö·ûINX¿Õ¸ñ
	 udp_data[3]<={"A","X","7","1"};   //´æ´¢×Ö·ûAV60	 
	 udp_data[4]<={"0","3","\r","\n"};   //´æ´¢×Ö·û45»»ÐÐ·û»Ø³µ·û                            

end //20 bytes

always@(posedge gmii_rx_clk)
begin	
  if(reset_n == 1'b0) begin
     ram_wr_finish <= 1'b0;
	  ram_addr_i <= 0;
	  ram_data_i <= 0;
	  i <= 0;
  end
  else begin
     if(i == 5) begin
        ram_wr_finish <= 1'b1;
        ram_wren_i<=1'b0;		  
     end
     else begin
        ram_wren_i <= 1'b1;
		  // ram_addr_i <= ram_addr_i+1'b1;
		  ram_data_i <= udp_data[i];
		  i <= i + 1'b1;
	  end
      
      if(ram_wren_i)begin
        ram_addr_i <= ram_addr_i+1'b1;
      end else begin
        ram_addr_i <= ram_addr_i;
      end
  end 
end 	


endmodule
