`include "mycpu.h"

module exe_stage(
    input                          clk           ,
    input                          reset         ,
    //allowin
    input                          ms_allowin    ,
    output                         es_allowin    ,
    //from ds
    input                          ds_to_es_valid,
    input  [`DS_TO_ES_BUS_WD -1:0] ds_to_es_bus  ,
    //to ms
    output                         es_to_ms_valid,
    output [`ES_TO_MS_BUS_WD -1:0] es_to_ms_bus  ,
    // data sram interface
    //�����ݴ洢�����н���
    output        data_sram_en   ,
    output [ 3:0] data_sram_wen  ,
    output [31:0] data_sram_addr ,
    output [31:0] data_sram_wdata,

    output [`ES_TO_DS_BUS_WD -1:0] es_to_ds_bus
);

//����δ����źţ����ź���ĩβ�����?
wire div_not_end          ;

reg         es_valid      ;
wire        es_ready_go   ;

reg  [`DS_TO_ES_BUS_WD -1:0] ds_to_es_bus_r;
wire [11:0] es_alu_op     ;
wire [3:0]  es_mult_div_op;
wire [3:0]  es_HILO_op    ;
wire        es_load_op    ;
wire        es_src1_is_sa ;  
wire        es_src1_is_pc ;
wire        es_src2_is_imm; 
wire        es_src2_is_immu; 
wire        es_src2_is_8  ;
wire        es_gr_we      ;
wire        es_mem_we     ;
wire [ 4:0] es_dest       ;
wire [15:0] es_imm        ;
wire [31:0] es_rs_value   ;
wire [31:0] es_rt_value   ;
wire [31:0] es_pc         ;
assign {es_alu_op      ,  //135:124
        es_mult_div_op ,
        es_HILO_op     ,
        es_load_op     ,  //123:123     ��ǰָ��Ϊlw
        es_src1_is_sa  ,  //122:122
        es_src1_is_pc  ,  //121:121
        es_src2_is_imm ,  //120:120
        es_src2_is_immu,
        es_src2_is_8   ,  //119:119
        es_gr_we       ,  //118:118     ��ǰָ��Ҫд�Ĵ���
        es_mem_we      ,  //117:117     ��ǰָ��ҪҪд�ڴ�
        es_dest        ,  //116:112     Ŀ�ļĴ���
        es_imm         ,  //111:96
        es_rs_value    ,  //95 :64
        es_rt_value    ,  //63 :32
        es_pc             //31 :0
       } = ds_to_es_bus_r;

wire [31:0] es_alu_src1   ;
wire [31:0] es_alu_src2   ;
wire [31:0] es_alu_result ;

wire        es_res_from_mem;

assign es_res_from_mem = es_load_op;

assign es_ready_go    = ~div_not_end;
assign es_allowin     = !es_valid || es_ready_go && ms_allowin;
assign es_to_ms_valid =  es_valid && es_ready_go;
always @(posedge clk) begin
    if (reset) begin
        es_valid <= 1'b0;
    end
    else if (es_allowin) begin
        es_valid <= ds_to_es_valid;
    end

    if (ds_to_es_valid && es_allowin) begin
        ds_to_es_bus_r <= ds_to_es_bus;
    end
end

assign es_alu_src1 = es_src1_is_sa  ? {27'b0, es_imm[10:6]} : 
                     es_src1_is_pc  ? es_pc[31:0] :
                                      es_rs_value;
assign es_alu_src2 = es_src2_is_imm ? {{16{es_imm[15]}}, es_imm[15:0]} : 
                     es_src2_is_immu ?{{16'h00}, es_imm[15:0]} :
                     es_src2_is_8   ? 32'd8 :
                                      es_rt_value;

alu u_alu(
    .alu_op     (es_alu_op    ),
    .alu_src1   (es_alu_src1  ),
    .alu_src2   (es_alu_src2  ),
    .alu_result (es_alu_result)
    );


//**************HILO����*************************//
wire wen_HI,wen_LO;
wire [31:0] wHI,wLO,rHI,rLO;
HILO hilo(
    .clk(clk),
    .wen_HI(wen_HI),
    .wHI(wHI),
    .wen_LO(wen_LO),
    .wLO(wLO),
    .rHI(rHI),
    .rLO(rLO)
);

//es_HILO_op 0:mfhi 1 : mflo ����˴�HILOget���ݵ��Ĵ���������ָ��
wire [31:0] es_alu_hilo_result =  ({32{es_HILO_op[0]                 }} & rHI)|
                                  ({32{es_HILO_op[1]                 }} & rLO)|
                                  ({32{~es_HILO_op[1]&~es_HILO_op[0]}} & es_alu_result);


assign es_to_ms_bus = {
    es_res_from_mem, //�ж�����Ҫ���ɵĽ�������Դ�?����������ALU
    es_gr_we,  
    es_dest, 
    es_alu_hilo_result, 
    es_pc 
    };


//����ǰ����ר�������ͻص�ID�������д�����ͻ��
assign es_to_ds_bus = {     es_valid&es_gr_we,
                            es_dest,
                            es_alu_hilo_result,
                            es_load_op&es_valid
                    };


//�ڴ��ڴ�������?�󲢲�����д��regfile  ����ͨ����ˮ��һֱ����WB��
assign data_sram_en    = 1'b1;
assign data_sram_wen   = es_mem_we&&es_valid ? 4'hf : 4'h0;
assign data_sram_addr  = es_alu_result;
assign data_sram_wdata = es_rt_value;


//***************ʵ�ֳ˷���********************//
wire [63:0] unsigned_prod,signed_prod;
assign unsigned_prod = es_rs_value * es_rt_value;
assign signed_prod = $signed(es_rs_value) * $signed(es_rt_value);

//***************ʵ�ֳ�����********************//
wire unsigned_divisor_tvalid,   signed_divisor_tvalid;
wire unsigned_dividend_tvalid,  signed_dividend_tvalid;
wire unsigned_divisor_tready,   signed_divisor_tready;
wire unsigned_dividend_tready,  signed_dividend_tready;
wire unsigned_divout_tvalid,    signed_divout_tvalid;
wire [63:0]unsigned_divout,     signed_divout;

wire divout_tvalid=unsigned_divout_tvalid|signed_divout_tvalid;
wire unsigned_div_handshake=unsigned_dividend_tready&unsigned_divisor_tready&
                            unsigned_dividend_tvalid&unsigned_divisor_tvalid;
wire signed_div_handshake  =signed_dividend_tready&signed_divisor_tready&
                            signed_dividend_tvalid&signed_divisor_tvalid;


reg get_div_source;
always @(posedge clk)begin
    //����������������������,���ǿ��ܻ�������es��������ÿ��es�л�һ��״̬��ʱ�������?
    //��allowin��ʱ�򣬵�ǰ����ˮ��������һ���������Ƿ���Ч������ǰ״̬һ������
    if(es_allowin)
        get_div_source<=0;
    //���ֳɹ�
    else if(unsigned_div_handshake|signed_div_handshake)
        get_div_source<=1;
end

reg save_div_end;
always @(posedge clk) begin
    if(es_allowin)
        save_div_end<=0;
    else if(divout_tvalid)
        save_div_end<=1;
end

assign unsigned_dividend_tvalid=~get_div_source&es_valid&es_mult_div_op[3];
assign unsigned_divisor_tvalid =~get_div_source&es_valid&es_mult_div_op[3];
assign signed_dividend_tvalid  =~get_div_source&es_valid&es_mult_div_op[2];
assign signed_divisor_tvalid   =~get_div_source&es_valid&es_mult_div_op[2];

div_unsigned DIV_unsigned(
    .aclk(clk),
    .s_axis_divisor_tvalid(unsigned_divisor_tvalid),
    .s_axis_divisor_tready(unsigned_divisor_tready), 
    .s_axis_divisor_tdata(es_rt_value),
    .s_axis_dividend_tvalid(unsigned_dividend_tvalid),
    .s_axis_dividend_tready(unsigned_dividend_tready), 
    .s_axis_dividend_tdata(es_rs_value),
    .m_axis_dout_tvalid(unsigned_divout_tvalid), 
    .m_axis_dout_tdata(unsigned_divout)
);

div_signed DIV_signed(
    .aclk(clk),
    .s_axis_divisor_tvalid(signed_divisor_tvalid),
    .s_axis_divisor_tready(signed_divisor_tready), 
    .s_axis_divisor_tdata(es_rt_value),
    .s_axis_dividend_tvalid(signed_dividend_tvalid),
    .s_axis_dividend_tready(signed_dividend_tready), 
    .s_axis_dividend_tdata(es_rs_value),
    .m_axis_dout_tvalid(signed_divout_tvalid), 
    .m_axis_dout_tdata(signed_divout)
);
assign div_not_end = es_valid & (es_mult_div_op[2]|es_mult_div_op[3])&
                    ~(save_div_end|divout_tvalid);

/**********************�������? HI LO�Ĵ���*******************/

//assign mult_div_op[0] = inst_mult;
//assign mult_div_op[1] = inst_multu;
//assign mult_div_op[2] = inst_div;
//assign mult_div_op[3] = inst_divu;
//assign HILO_op[0]     = inst_mfhi;
//assign HILO_op[1]     = inst_mflo;
//assign HILO_op[2]     = inst_mthi;
//assign HILO_op[3]     = isnt_mtlo;

assign wHI =  ({32{es_mult_div_op[0]}}&signed_prod[63:32]) |
            ({32{es_mult_div_op[1]}}&unsigned_prod[63:32]) |
            ({32{es_mult_div_op[2]}}&signed_divout[31:0])|
            ({32{es_mult_div_op[3]}}&unsigned_divout[31:0])|
            ({32{es_HILO_op[2]}}&es_rs_value);
assign wLO =  ({32{es_mult_div_op[0]}}&signed_prod[31:0]) |
            ({32{es_mult_div_op[1]}}&unsigned_prod[31:0]) |
            ({32{es_mult_div_op[2]}}&signed_divout[63:32])|
            ({32{es_mult_div_op[3]}}&unsigned_divout[63:32])|
            ({32{es_HILO_op[3]}}&es_rs_value);
assign wen_HI = es_valid & 
            (es_HILO_op[2]|es_mult_div_op[0]|es_mult_div_op[1]|
                    (es_mult_div_op[2]&signed_divout_tvalid)|
                    (es_mult_div_op[3]&unsigned_divout_tvalid));

assign wen_LO = es_valid & 
            (es_HILO_op[3]|es_mult_div_op[0]|es_mult_div_op[1]|
                    (es_mult_div_op[2]&signed_divout_tvalid)|
                    (es_mult_div_op[3]&unsigned_divout_tvalid));


endmodule