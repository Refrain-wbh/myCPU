`include "mycpu.h"

module id_stage(
    input                          clk           ,
    input                          reset         ,
    //allowin
    input                          es_allowin    ,
    output                         ds_allowin    ,
    //from fs
    input                          fs_to_ds_valid,
    //里面是上一个阶段触发器中的pc，和pc对应的inst
    input  [`FS_TO_DS_BUS_WD -1:0] fs_to_ds_bus  ,
    //to es
    output                         ds_to_es_valid,
    output [`DS_TO_ES_BUS_WD -1:0] ds_to_es_bus  ,
    //to fs
    output [`BR_BUS_WD       -1:0] br_bus        ,
    //to rf: for write back
    input  [`WS_TO_RF_BUS_WD -1:0] ws_to_rf_bus  ,
    //dest from after 用于消除冲突
    input  [`ES_TO_DS_BUS_WD -1:0] es_to_ds_bus,
    input  [`MS_TO_DS_BUS_WD -1:0] ms_to_ds_bus,
    input  [`WS_TO_DS_BUS_WD -1:0] ws_to_ds_bus
);
//sa域（R类型指令）指示的是移位操作的位数，其他情况下都为0
//SRA SLL等移位指令，SLL rd,rt,sa 表示的是sa指定位移量，rt进行位移，此时rs为0
//核心要点是抓住bus的数据传输，以及这一层对bus数据的加工

//数据前递，包括往后三级的将要写入的数据，写书使能信号
//EXE级要传回是否是内存操作，如果是的话那么可能要阻塞一个周期才能得出前递结果
wire pro_EXE_wena;
wire pro_MEM_wena;
wire pro_WB_wena;

wire [4:0]pro_EXE_dest;
wire [4:0]pro_MEM_dest;
wire [4:0]pro_WB_dest;

wire [31:0]pro_EXE_result;
wire [31:0]pro_MEM_result;
wire [31:0]pro_WB_result;

//这个在尾部进行判断
wire pro_EXE_loading;


assign {pro_EXE_wena,pro_EXE_dest,pro_EXE_result,pro_EXE_loading}=es_to_ds_bus;
assign {pro_MEM_wena,pro_MEM_dest,pro_MEM_result}=ms_to_ds_bus;
assign {pro_WB_wena, pro_WB_dest, pro_WB_result} =ws_to_ds_bus;

//这个在最后才赋值
wire if_EXE_notloading;



reg         ds_valid   ;
wire        ds_ready_go;

wire [31                 :0] fs_pc;
reg  [`FS_TO_DS_BUS_WD -1:0] fs_to_ds_bus_r;
assign fs_pc = fs_to_ds_bus[31:0];

wire [31:0] ds_inst;
wire [31:0] ds_pc  ;
assign {ds_inst,
        ds_pc  } = fs_to_ds_bus_r;

wire        rf_we   ;
wire [ 4:0] rf_waddr;
wire [31:0] rf_wdata;
assign {rf_we   ,  //37:37
        rf_waddr,  //36:32
        rf_wdata   //31:0
       } = ws_to_rf_bus;

wire        br_taken;
wire [31:0] br_target;

wire [11:0] alu_op;
wire [3:0] mult_div_op;
wire [3:0] HILO_op;
wire        load_op;
//src1_is sa 的情况出现在SLL，SLA等移位指令
wire        src1_is_sa;
wire        src1_is_pc;
wire        src2_is_imm;
wire        src2_is_immu;
wire        src2_is_8;
wire        res_from_mem;
wire        gr_we;
wire        mem_we;



wire [ 4:0] dest;
wire [15:0] imm;
wire [31:0] rs_value;
wire [31:0] rt_value;

wire [ 5:0] op;
wire [ 4:0] rs;
wire [ 4:0] rt;
wire [ 4:0] rd;
wire [ 4:0] sa;
wire [ 5:0] func;
wire [25:0] jidx;
wire [63:0] op_d;
wire [31:0] rs_d;
wire [31:0] rt_d;
wire [31:0] rd_d;
wire [31:0] sa_d;
wire [63:0] func_d;


wire        dst_is_r31;  
wire        dst_is_rt;   

wire [ 4:0] rf_raddr1;
wire [31:0] rf_rdata1;
wire [ 4:0] rf_raddr2;
wire [31:0] rf_rdata2;

wire        rs_eq_rt;

assign br_bus       = {br_taken,br_target};

assign ds_to_es_bus = {alu_op      ,  //135:124
                       mult_div_op ,
                       HILO_op     ,
                       load_op     ,  //123:123     当前指令为lw
                       src1_is_sa  ,  //122:122
                       src1_is_pc  ,  //121:121
                       src2_is_imm ,
                       src2_is_immu,  //120:120
                       src2_is_8   ,  //119:119
                       gr_we       ,  //118:118     当前指令要写寄存器
                       mem_we      ,  //117:117     当前指令要要写内存
                       dest        ,  //116:112     目的寄存器
                       imm         ,  //111:96
                       rs_value    ,  //95 :64
                       rt_value    ,  //63 :32
                       ds_pc          //31 :0
                      };


assign ds_ready_go    = if_EXE_notloading;
assign ds_allowin     = !ds_valid || ds_ready_go && es_allowin;
assign ds_to_es_valid = ds_valid && ds_ready_go;
always @(posedge clk) begin
    if(reset)begin
        ds_valid<=1'b0;
    end
    else if(ds_allowin)begin
        ds_valid<=fs_to_ds_valid;
    end

    if (fs_to_ds_valid && ds_allowin) begin
        fs_to_ds_bus_r <= fs_to_ds_bus;
    end
end

assign op   = ds_inst[31:26];
assign rs   = ds_inst[25:21];
assign rt   = ds_inst[20:16];
assign rd   = ds_inst[15:11];
assign sa   = ds_inst[10: 6];
assign func = ds_inst[ 5: 0];
assign imm  = ds_inst[15: 0];
assign jidx = ds_inst[25: 0];

decoder_6_64 u_dec0(.in(op  ), .out(op_d  ));
decoder_6_64 u_dec1(.in(func), .out(func_d));
decoder_5_32 u_dec2(.in(rs  ), .out(rs_d  ));
decoder_5_32 u_dec3(.in(rt  ), .out(rt_d  ));
decoder_5_32 u_dec4(.in(rd  ), .out(rd_d  ));
decoder_5_32 u_dec5(.in(sa  ), .out(sa_d  ));

wire inst_add    = op_d[6'h00] & func_d[6'h20] & sa_d[5'h00];
wire inst_addi   = op_d[6'h08];
wire inst_addu   = op_d[6'h00] & func_d[6'h21] & sa_d[5'h00];
wire inst_addiu  = op_d[6'h09];
wire inst_sub    = op_d[6'h00] & func_d[6'h22] & sa_d[5'h00];
wire inst_subu   = op_d[6'h00] & func_d[6'h23] & sa_d[5'h00];
wire inst_slt    = op_d[6'h00] & func_d[6'h2a] & sa_d[5'h00];
wire inst_slti   = op_d[6'h0a];
wire inst_sltu   = op_d[6'h00] & func_d[6'h2b] & sa_d[5'h00];
wire inst_sltiu  = op_d[6'h0b];
wire inst_and    = op_d[6'h00] & func_d[6'h24] & sa_d[5'h00];
wire inst_andi   = op_d[6'h0c];
wire inst_lui    = op_d[6'h0f] & rs_d[5'h00];
wire inst_nor    = op_d[6'h00] & func_d[6'h27] & sa_d[5'h00];
wire inst_or     = op_d[6'h00] & func_d[6'h25] & sa_d[5'h00];
wire inst_ori    = op_d[6'h0d];
wire inst_xor    = op_d[6'h00] & func_d[6'h26] & sa_d[5'h00];
wire inst_xori   = op_d[6'h0e];
wire inst_sllv   = op_d[6'h00] & func_d[6'h04] & sa_d[5'h00];
wire inst_sll    = op_d[6'h00] & func_d[6'h00] & rs_d[5'h00];
wire inst_srav   = op_d[6'h00] & func_d[6'h07] & sa_d[5'h00];
wire inst_sra    = op_d[6'h00] & func_d[6'h03] & rs_d[5'h00];
wire inst_srl    = op_d[6'h00] & func_d[6'h02] & rs_d[5'h00];
wire inst_srlv   = op_d[6'h00] & func_d[6'h06] & sa_d[5'h00];

wire inst_lw     = op_d[6'h23];
wire inst_sw     = op_d[6'h2b];
wire inst_beq    = op_d[6'h04];
wire inst_bne    = op_d[6'h05];
wire inst_jal    = op_d[6'h03];
wire inst_jr     = op_d[6'h00] & func_d[6'h08] & rt_d[5'h00] & rd_d[5'h00] & sa_d[5'h00];

wire inst_div    = op_d[6'h00] & func_d[6'h1a] & rd_d[5'h00] & sa_d[5'h00];
wire inst_divu   = op_d[6'h00] & func_d[6'h1b] & rd_d[5'h00] & sa_d[5'h00];
wire inst_mult   = op_d[6'h00] & func_d[6'h18] & rd_d[5'h00] & sa_d[5'h00];
wire inst_multu  = op_d[6'h00] & func_d[6'h19] & rd_d[5'h00] & sa_d[5'h00];

wire inst_mfhi   = op_d[6'h00] & func_d[6'h10] & rs_d[5'h00] & rt_d[5'h00] & sa_d[5'h00];
wire inst_mflo   = op_d[6'h00] & func_d[6'h12] & rs_d[5'h00] & rt_d[5'h00] & sa_d[5'h00];
wire inst_mthi   = op_d[6'h00] & func_d[6'h11] & rt_d[5'h00] & rd_d[5'h00] & sa_d[5'h00];
wire inst_mtlo   = op_d[6'h00] & func_d[6'h13] & rt_d[5'h00] & rd_d[5'h00] & sa_d[5'h00];

assign alu_op[ 0] = inst_addi | inst_add | inst_addu | inst_addiu | inst_lw | inst_sw | inst_jal;
assign alu_op[ 1] = inst_sub | inst_subu;
assign alu_op[ 2] = inst_slt | inst_slti;
assign alu_op[ 3] = inst_sltu | inst_sltiu;
assign alu_op[ 4] = inst_and | inst_andi;
assign alu_op[ 5] = inst_nor;
assign alu_op[ 6] = inst_or | inst_ori;
assign alu_op[ 7] = inst_xor | inst_xori;
assign alu_op[ 8] = inst_sll | inst_sllv;
assign alu_op[ 9] = inst_srl | inst_srlv;
assign alu_op[10] = inst_sra | inst_srav;
assign alu_op[11] = inst_lui;

assign mult_div_op[0] = inst_mult;
assign mult_div_op[1] = inst_multu;
assign mult_div_op[2] = inst_div;
assign mult_div_op[3] = inst_divu;

assign HILO_op[0]     = inst_mfhi;
assign HILO_op[1]     = inst_mflo;
assign HILO_op[2]     = inst_mthi;
assign HILO_op[3]     = inst_mtlo;







//对操作类型进行解析
//src1_is sa 的情况出现在SLL，SLA等移位指令
assign src1_is_sa   = inst_sll   | inst_srl | inst_sra;
assign src1_is_pc   = inst_jal;
assign src2_is_imm  = inst_sltiu | inst_slti | inst_addi | inst_addiu | 
                    inst_lui | inst_lw | inst_sw;
assign src2_is_immu = inst_andi | inst_ori | inst_xori;
assign src2_is_8    = inst_jal;
assign res_from_mem = inst_lw;
assign dst_is_r31   = inst_jal;
assign dst_is_rt    = inst_sltiu | inst_slti | inst_addi | 
                    inst_addiu | inst_lui | inst_lw | 
                    inst_andi | inst_ori | inst_xori;
assign gr_we        = ~inst_sw & ~inst_beq & ~inst_bne & ~inst_jr &
                          ~inst_div & ~inst_divu & ~inst_mult & ~inst_multu &
                          ~inst_mthi & ~inst_mtlo;
assign mem_we       = inst_sw;
assign load_op      = inst_lw;


assign dest         = dst_is_r31 ? 5'd31 :
                      dst_is_rt  ? rt    : 
                                   rd;

assign rf_raddr1 = rs;
assign rf_raddr2 = rt;
regfile u_regfile(
    .clk    (clk      ),
    .raddr1 (rf_raddr1),
    .rdata1 (rf_rdata1),
    .raddr2 (rf_raddr2),
    .rdata2 (rf_rdata2),
    .we     (rf_we    ),
    .waddr  (rf_waddr ),
    .wdata  (rf_wdata )
    );

//这个逻辑是四选一的，即读出的值不仅仅是寄存器的值，还可能是来自后面三级的值
assign rs_value = pro_EXE_wena&&!(pro_EXE_dest^rf_raddr1)?pro_EXE_result:
                  pro_MEM_wena&&!(pro_MEM_dest^rf_raddr1)?pro_MEM_result:
                  pro_WB_wena&&!(pro_WB_dest^rf_raddr1)?pro_WB_result:
                                                        rf_rdata1;
assign rt_value = pro_EXE_wena&&!(pro_EXE_dest^rf_raddr2)?pro_EXE_result:
                  pro_MEM_wena&&!(pro_MEM_dest^rf_raddr2)?pro_MEM_result:
                  pro_WB_wena&&!(pro_WB_dest^rf_raddr2)?pro_WB_result:
                                                        rf_rdata2;

assign rs_eq_rt = (rs_value == rt_value);
assign br_taken = (   inst_beq  &&  rs_eq_rt
                   || inst_bne  && !rs_eq_rt
                   || inst_jal
                   || inst_jr
                  ) && ds_valid;
assign br_target = (inst_beq || inst_bne) ? (fs_pc + {{14{imm[15]}}, imm[15:0], 2'b0}) :
                   (inst_jr)              ? rs_value :
                  /*inst_jal*/              {fs_pc[31:28], jidx[25:0], 2'b0};

//判断EXE不是loading，或者与ID级不冲突 或者写入的目标为0号寄存器
assign if_EXE_notloading=!pro_EXE_loading || 
                    ((pro_EXE_dest^rf_raddr1)&&(pro_EXE_dest^rf_raddr2))||
                    !pro_EXE_dest;



endmodule

