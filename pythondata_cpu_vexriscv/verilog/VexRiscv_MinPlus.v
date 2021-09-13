// Generator : SpinalHDL v1.3.6    git head : 9bf01e7f360e003fac1dd5ca8b8f4bffec0e52b8
// Date      : 13/09/2021, 23:06:44
// Component : VexRiscv


`define Src1CtrlEnum_defaultEncoding_type [1:0]
`define Src1CtrlEnum_defaultEncoding_RS 2'b00
`define Src1CtrlEnum_defaultEncoding_IMU 2'b01
`define Src1CtrlEnum_defaultEncoding_PC_INCREMENT 2'b10
`define Src1CtrlEnum_defaultEncoding_URS1 2'b11

`define ShiftCtrlEnum_defaultEncoding_type [1:0]
`define ShiftCtrlEnum_defaultEncoding_DISABLE_1 2'b00
`define ShiftCtrlEnum_defaultEncoding_SLL_1 2'b01
`define ShiftCtrlEnum_defaultEncoding_SRL_1 2'b10
`define ShiftCtrlEnum_defaultEncoding_SRA_1 2'b11

`define AluCtrlEnum_defaultEncoding_type [1:0]
`define AluCtrlEnum_defaultEncoding_ADD_SUB 2'b00
`define AluCtrlEnum_defaultEncoding_SLT_SLTU 2'b01
`define AluCtrlEnum_defaultEncoding_BITWISE 2'b10

`define BranchCtrlEnum_defaultEncoding_type [1:0]
`define BranchCtrlEnum_defaultEncoding_INC 2'b00
`define BranchCtrlEnum_defaultEncoding_B 2'b01
`define BranchCtrlEnum_defaultEncoding_JAL 2'b10
`define BranchCtrlEnum_defaultEncoding_JALR 2'b11

`define EnvCtrlEnum_defaultEncoding_type [1:0]
`define EnvCtrlEnum_defaultEncoding_NONE 2'b00
`define EnvCtrlEnum_defaultEncoding_XRET 2'b01
`define EnvCtrlEnum_defaultEncoding_ECALL 2'b10

`define AluBitwiseCtrlEnum_defaultEncoding_type [1:0]
`define AluBitwiseCtrlEnum_defaultEncoding_XOR_1 2'b00
`define AluBitwiseCtrlEnum_defaultEncoding_OR_1 2'b01
`define AluBitwiseCtrlEnum_defaultEncoding_AND_1 2'b10

`define Src2CtrlEnum_defaultEncoding_type [1:0]
`define Src2CtrlEnum_defaultEncoding_RS 2'b00
`define Src2CtrlEnum_defaultEncoding_IMI 2'b01
`define Src2CtrlEnum_defaultEncoding_IMS 2'b10
`define Src2CtrlEnum_defaultEncoding_PC 2'b11

module StreamFifoLowLatency (
      input   io_push_valid,
      output  io_push_ready,
      input   io_push_payload_error,
      input  [31:0] io_push_payload_inst,
      output reg  io_pop_valid,
      input   io_pop_ready,
      output reg  io_pop_payload_error,
      output reg [31:0] io_pop_payload_inst,
      input   io_flush,
      output [0:0] io_occupancy,
      input   clk,
      input   reset);
  wire  _zz_5_;
  wire [0:0] _zz_6_;
  reg  _zz_1_;
  reg  pushPtr_willIncrement;
  reg  pushPtr_willClear;
  wire  pushPtr_willOverflowIfInc;
  wire  pushPtr_willOverflow;
  reg  popPtr_willIncrement;
  reg  popPtr_willClear;
  wire  popPtr_willOverflowIfInc;
  wire  popPtr_willOverflow;
  wire  ptrMatch;
  reg  risingOccupancy;
  wire  empty;
  wire  full;
  wire  pushing;
  wire  popping;
  wire [32:0] _zz_2_;
  wire [32:0] _zz_3_;
  reg [32:0] _zz_4_;
  assign _zz_5_ = (! empty);
  assign _zz_6_ = _zz_2_[0 : 0];
  always @ (*) begin
    _zz_1_ = 1'b0;
    if(pushing)begin
      _zz_1_ = 1'b1;
    end
  end

  always @ (*) begin
    pushPtr_willIncrement = 1'b0;
    if(pushing)begin
      pushPtr_willIncrement = 1'b1;
    end
  end

  always @ (*) begin
    pushPtr_willClear = 1'b0;
    if(io_flush)begin
      pushPtr_willClear = 1'b1;
    end
  end

  assign pushPtr_willOverflowIfInc = 1'b1;
  assign pushPtr_willOverflow = (pushPtr_willOverflowIfInc && pushPtr_willIncrement);
  always @ (*) begin
    popPtr_willIncrement = 1'b0;
    if(popping)begin
      popPtr_willIncrement = 1'b1;
    end
  end

  always @ (*) begin
    popPtr_willClear = 1'b0;
    if(io_flush)begin
      popPtr_willClear = 1'b1;
    end
  end

  assign popPtr_willOverflowIfInc = 1'b1;
  assign popPtr_willOverflow = (popPtr_willOverflowIfInc && popPtr_willIncrement);
  assign ptrMatch = 1'b1;
  assign empty = (ptrMatch && (! risingOccupancy));
  assign full = (ptrMatch && risingOccupancy);
  assign pushing = (io_push_valid && io_push_ready);
  assign popping = (io_pop_valid && io_pop_ready);
  assign io_push_ready = (! full);
  always @ (*) begin
    if(_zz_5_)begin
      io_pop_valid = 1'b1;
    end else begin
      io_pop_valid = io_push_valid;
    end
  end

  assign _zz_2_ = _zz_3_;
  always @ (*) begin
    if(_zz_5_)begin
      io_pop_payload_error = _zz_6_[0];
    end else begin
      io_pop_payload_error = io_push_payload_error;
    end
  end

  always @ (*) begin
    if(_zz_5_)begin
      io_pop_payload_inst = _zz_2_[32 : 1];
    end else begin
      io_pop_payload_inst = io_push_payload_inst;
    end
  end

  assign io_occupancy = (risingOccupancy && ptrMatch);
  assign _zz_3_ = _zz_4_;
  always @ (posedge clk) begin
    if(reset) begin
      risingOccupancy <= 1'b0;
    end else begin
      if((pushing != popping))begin
        risingOccupancy <= pushing;
      end
      if(io_flush)begin
        risingOccupancy <= 1'b0;
      end
    end
  end

  always @ (posedge clk) begin
    if(_zz_1_)begin
      _zz_4_ <= {io_push_payload_inst,io_push_payload_error};
    end
  end

endmodule

module VexRiscv (
      input  [31:0] externalResetVector,
      input   timerInterrupt,
      input   softwareInterrupt,
      input  [31:0] externalInterruptArray,
      output  iBusWishbone_CYC,
      output  iBusWishbone_STB,
      input   iBusWishbone_ACK,
      output  iBusWishbone_WE,
      output [29:0] iBusWishbone_ADR,
      input  [31:0] iBusWishbone_DAT_MISO,
      output [31:0] iBusWishbone_DAT_MOSI,
      output [3:0] iBusWishbone_SEL,
      input   iBusWishbone_ERR,
      output [1:0] iBusWishbone_BTE,
      output [2:0] iBusWishbone_CTI,
      output  dBusWishbone_CYC,
      output  dBusWishbone_STB,
      input   dBusWishbone_ACK,
      output  dBusWishbone_WE,
      output [29:0] dBusWishbone_ADR,
      input  [31:0] dBusWishbone_DAT_MISO,
      output [31:0] dBusWishbone_DAT_MOSI,
      output reg [3:0] dBusWishbone_SEL,
      input   dBusWishbone_ERR,
      output [1:0] dBusWishbone_BTE,
      output [2:0] dBusWishbone_CTI,
      input   clk,
      input   reset);
  reg [31:0] _zz_166_;
  reg [31:0] _zz_167_;
  reg [31:0] _zz_168_;
  wire  IBusSimplePlugin_rspJoin_rspBuffer_c_io_push_ready;
  wire  IBusSimplePlugin_rspJoin_rspBuffer_c_io_pop_valid;
  wire  IBusSimplePlugin_rspJoin_rspBuffer_c_io_pop_payload_error;
  wire [31:0] IBusSimplePlugin_rspJoin_rspBuffer_c_io_pop_payload_inst;
  wire [0:0] IBusSimplePlugin_rspJoin_rspBuffer_c_io_occupancy;
  wire  _zz_169_;
  wire  _zz_170_;
  wire  _zz_171_;
  wire  _zz_172_;
  wire  _zz_173_;
  wire [1:0] _zz_174_;
  wire  _zz_175_;
  wire  _zz_176_;
  wire  _zz_177_;
  wire  _zz_178_;
  wire  _zz_179_;
  wire  _zz_180_;
  wire  _zz_181_;
  wire  _zz_182_;
  wire  _zz_183_;
  wire  _zz_184_;
  wire  _zz_185_;
  wire  _zz_186_;
  wire  _zz_187_;
  wire  _zz_188_;
  wire  _zz_189_;
  wire  _zz_190_;
  wire  _zz_191_;
  wire [1:0] _zz_192_;
  wire  _zz_193_;
  wire [3:0] _zz_194_;
  wire [2:0] _zz_195_;
  wire [31:0] _zz_196_;
  wire [2:0] _zz_197_;
  wire [0:0] _zz_198_;
  wire [2:0] _zz_199_;
  wire [0:0] _zz_200_;
  wire [2:0] _zz_201_;
  wire [0:0] _zz_202_;
  wire [2:0] _zz_203_;
  wire [0:0] _zz_204_;
  wire [2:0] _zz_205_;
  wire [2:0] _zz_206_;
  wire [0:0] _zz_207_;
  wire [0:0] _zz_208_;
  wire [0:0] _zz_209_;
  wire [0:0] _zz_210_;
  wire [0:0] _zz_211_;
  wire [0:0] _zz_212_;
  wire [0:0] _zz_213_;
  wire [0:0] _zz_214_;
  wire [0:0] _zz_215_;
  wire [0:0] _zz_216_;
  wire [0:0] _zz_217_;
  wire [0:0] _zz_218_;
  wire [2:0] _zz_219_;
  wire [4:0] _zz_220_;
  wire [11:0] _zz_221_;
  wire [11:0] _zz_222_;
  wire [31:0] _zz_223_;
  wire [31:0] _zz_224_;
  wire [31:0] _zz_225_;
  wire [31:0] _zz_226_;
  wire [31:0] _zz_227_;
  wire [31:0] _zz_228_;
  wire [31:0] _zz_229_;
  wire [32:0] _zz_230_;
  wire [31:0] _zz_231_;
  wire [32:0] _zz_232_;
  wire [19:0] _zz_233_;
  wire [11:0] _zz_234_;
  wire [11:0] _zz_235_;
  wire [1:0] _zz_236_;
  wire [1:0] _zz_237_;
  wire [1:0] _zz_238_;
  wire [1:0] _zz_239_;
  wire [0:0] _zz_240_;
  wire [0:0] _zz_241_;
  wire [0:0] _zz_242_;
  wire [0:0] _zz_243_;
  wire [0:0] _zz_244_;
  wire [0:0] _zz_245_;
  wire [6:0] _zz_246_;
  wire  _zz_247_;
  wire  _zz_248_;
  wire [1:0] _zz_249_;
  wire  _zz_250_;
  wire  _zz_251_;
  wire [31:0] _zz_252_;
  wire [31:0] _zz_253_;
  wire  _zz_254_;
  wire [2:0] _zz_255_;
  wire [2:0] _zz_256_;
  wire  _zz_257_;
  wire [0:0] _zz_258_;
  wire [19:0] _zz_259_;
  wire [31:0] _zz_260_;
  wire [31:0] _zz_261_;
  wire  _zz_262_;
  wire  _zz_263_;
  wire  _zz_264_;
  wire [0:0] _zz_265_;
  wire [0:0] _zz_266_;
  wire [0:0] _zz_267_;
  wire [0:0] _zz_268_;
  wire [1:0] _zz_269_;
  wire [1:0] _zz_270_;
  wire  _zz_271_;
  wire [0:0] _zz_272_;
  wire [16:0] _zz_273_;
  wire [31:0] _zz_274_;
  wire [31:0] _zz_275_;
  wire [31:0] _zz_276_;
  wire [31:0] _zz_277_;
  wire [31:0] _zz_278_;
  wire [31:0] _zz_279_;
  wire [31:0] _zz_280_;
  wire [31:0] _zz_281_;
  wire [31:0] _zz_282_;
  wire  _zz_283_;
  wire [0:0] _zz_284_;
  wire [0:0] _zz_285_;
  wire [0:0] _zz_286_;
  wire [0:0] _zz_287_;
  wire  _zz_288_;
  wire [0:0] _zz_289_;
  wire [14:0] _zz_290_;
  wire [31:0] _zz_291_;
  wire [31:0] _zz_292_;
  wire [31:0] _zz_293_;
  wire [31:0] _zz_294_;
  wire [31:0] _zz_295_;
  wire  _zz_296_;
  wire [0:0] _zz_297_;
  wire [0:0] _zz_298_;
  wire  _zz_299_;
  wire [0:0] _zz_300_;
  wire [11:0] _zz_301_;
  wire [31:0] _zz_302_;
  wire [31:0] _zz_303_;
  wire [31:0] _zz_304_;
  wire [31:0] _zz_305_;
  wire [31:0] _zz_306_;
  wire [31:0] _zz_307_;
  wire [0:0] _zz_308_;
  wire [0:0] _zz_309_;
  wire [1:0] _zz_310_;
  wire [1:0] _zz_311_;
  wire  _zz_312_;
  wire [0:0] _zz_313_;
  wire [7:0] _zz_314_;
  wire [31:0] _zz_315_;
  wire [31:0] _zz_316_;
  wire [31:0] _zz_317_;
  wire  _zz_318_;
  wire  _zz_319_;
  wire  _zz_320_;
  wire [0:0] _zz_321_;
  wire [0:0] _zz_322_;
  wire  _zz_323_;
  wire [0:0] _zz_324_;
  wire [4:0] _zz_325_;
  wire [31:0] _zz_326_;
  wire  _zz_327_;
  wire  _zz_328_;
  wire  _zz_329_;
  wire [0:0] _zz_330_;
  wire [0:0] _zz_331_;
  wire  _zz_332_;
  wire [0:0] _zz_333_;
  wire [1:0] _zz_334_;
  wire [31:0] _zz_335_;
  wire  _zz_336_;
  wire [0:0] _zz_337_;
  wire [1:0] _zz_338_;
  wire [0:0] _zz_339_;
  wire [4:0] _zz_340_;
  wire [1:0] _zz_341_;
  wire [1:0] _zz_342_;
  wire [0:0] _zz_343_;
  wire [0:0] _zz_344_;
  wire [31:0] _zz_345_;
  wire [31:0] _zz_346_;
  wire [31:0] _zz_347_;
  wire [31:0] _zz_348_;
  wire [31:0] _zz_349_;
  wire [31:0] _zz_350_;
  wire [31:0] _zz_351_;
  wire  _zz_352_;
  wire [0:0] _zz_353_;
  wire [1:0] _zz_354_;
  wire [31:0] _zz_355_;
  wire [31:0] _zz_356_;
  wire [31:0] _zz_357_;
  wire [31:0] _zz_358_;
  wire [31:0] _zz_359_;
  wire [31:0] _zz_360_;
  wire  _zz_361_;
  wire [0:0] _zz_362_;
  wire [12:0] _zz_363_;
  wire [31:0] _zz_364_;
  wire [31:0] _zz_365_;
  wire [31:0] _zz_366_;
  wire  _zz_367_;
  wire [0:0] _zz_368_;
  wire [6:0] _zz_369_;
  wire [31:0] _zz_370_;
  wire [31:0] _zz_371_;
  wire [31:0] _zz_372_;
  wire  _zz_373_;
  wire [0:0] _zz_374_;
  wire [0:0] _zz_375_;
  wire `Src1CtrlEnum_defaultEncoding_type decode_SRC1_CTRL;
  wire `Src1CtrlEnum_defaultEncoding_type _zz_1_;
  wire `Src1CtrlEnum_defaultEncoding_type _zz_2_;
  wire `Src1CtrlEnum_defaultEncoding_type _zz_3_;
  wire  decode_CSR_WRITE_OPCODE;
  wire [31:0] memory_MEMORY_READ_DATA;
  wire `ShiftCtrlEnum_defaultEncoding_type _zz_4_;
  wire `ShiftCtrlEnum_defaultEncoding_type _zz_5_;
  wire `ShiftCtrlEnum_defaultEncoding_type decode_SHIFT_CTRL;
  wire `ShiftCtrlEnum_defaultEncoding_type _zz_6_;
  wire `ShiftCtrlEnum_defaultEncoding_type _zz_7_;
  wire `ShiftCtrlEnum_defaultEncoding_type _zz_8_;
  wire  decode_SRC2_FORCE_ZERO;
  wire `AluCtrlEnum_defaultEncoding_type decode_ALU_CTRL;
  wire `AluCtrlEnum_defaultEncoding_type _zz_9_;
  wire `AluCtrlEnum_defaultEncoding_type _zz_10_;
  wire `AluCtrlEnum_defaultEncoding_type _zz_11_;
  wire `BranchCtrlEnum_defaultEncoding_type decode_BRANCH_CTRL;
  wire `BranchCtrlEnum_defaultEncoding_type _zz_12_;
  wire `BranchCtrlEnum_defaultEncoding_type _zz_13_;
  wire `BranchCtrlEnum_defaultEncoding_type _zz_14_;
  wire  decode_IS_CSR;
  wire [31:0] writeBack_FORMAL_PC_NEXT;
  wire [31:0] memory_FORMAL_PC_NEXT;
  wire [31:0] execute_FORMAL_PC_NEXT;
  wire [31:0] decode_FORMAL_PC_NEXT;
  wire `EnvCtrlEnum_defaultEncoding_type _zz_15_;
  wire `EnvCtrlEnum_defaultEncoding_type _zz_16_;
  wire `EnvCtrlEnum_defaultEncoding_type _zz_17_;
  wire `EnvCtrlEnum_defaultEncoding_type _zz_18_;
  wire `EnvCtrlEnum_defaultEncoding_type decode_ENV_CTRL;
  wire `EnvCtrlEnum_defaultEncoding_type _zz_19_;
  wire `EnvCtrlEnum_defaultEncoding_type _zz_20_;
  wire `EnvCtrlEnum_defaultEncoding_type _zz_21_;
  wire  decode_CSR_READ_OPCODE;
  wire [1:0] memory_MEMORY_ADDRESS_LOW;
  wire [1:0] execute_MEMORY_ADDRESS_LOW;
  wire  decode_BYPASSABLE_EXECUTE_STAGE;
  wire  decode_SRC_LESS_UNSIGNED;
  wire [31:0] writeBack_REGFILE_WRITE_DATA;
  wire [31:0] execute_REGFILE_WRITE_DATA;
  wire [31:0] execute_SHIFT_RIGHT;
  wire [31:0] decode_RS2;
  wire [31:0] execute_BRANCH_CALC;
  wire  decode_MEMORY_STORE;
  wire  execute_BRANCH_DO;
  wire [31:0] decode_RS1;
  wire  execute_BYPASSABLE_MEMORY_STAGE;
  wire  decode_BYPASSABLE_MEMORY_STAGE;
  wire `AluBitwiseCtrlEnum_defaultEncoding_type decode_ALU_BITWISE_CTRL;
  wire `AluBitwiseCtrlEnum_defaultEncoding_type _zz_22_;
  wire `AluBitwiseCtrlEnum_defaultEncoding_type _zz_23_;
  wire `AluBitwiseCtrlEnum_defaultEncoding_type _zz_24_;
  wire `Src2CtrlEnum_defaultEncoding_type decode_SRC2_CTRL;
  wire `Src2CtrlEnum_defaultEncoding_type _zz_25_;
  wire `Src2CtrlEnum_defaultEncoding_type _zz_26_;
  wire `Src2CtrlEnum_defaultEncoding_type _zz_27_;
  reg [31:0] _zz_28_;
  wire  execute_CSR_READ_OPCODE;
  wire  execute_CSR_WRITE_OPCODE;
  wire  execute_IS_CSR;
  wire `EnvCtrlEnum_defaultEncoding_type memory_ENV_CTRL;
  wire `EnvCtrlEnum_defaultEncoding_type _zz_29_;
  wire `EnvCtrlEnum_defaultEncoding_type execute_ENV_CTRL;
  wire `EnvCtrlEnum_defaultEncoding_type _zz_30_;
  wire  _zz_31_;
  wire  _zz_32_;
  wire `EnvCtrlEnum_defaultEncoding_type writeBack_ENV_CTRL;
  wire `EnvCtrlEnum_defaultEncoding_type _zz_33_;
  wire [31:0] memory_BRANCH_CALC;
  wire  memory_BRANCH_DO;
  wire [31:0] _zz_34_;
  wire [31:0] execute_PC;
  wire [31:0] execute_RS1;
  wire `BranchCtrlEnum_defaultEncoding_type execute_BRANCH_CTRL;
  wire `BranchCtrlEnum_defaultEncoding_type _zz_35_;
  wire  _zz_36_;
  wire  decode_RS2_USE;
  wire  decode_RS1_USE;
  wire  execute_REGFILE_WRITE_VALID;
  wire  execute_BYPASSABLE_EXECUTE_STAGE;
  wire  memory_REGFILE_WRITE_VALID;
  wire [31:0] memory_INSTRUCTION;
  wire  memory_BYPASSABLE_MEMORY_STAGE;
  wire  writeBack_REGFILE_WRITE_VALID;
  wire [31:0] memory_SHIFT_RIGHT;
  reg [31:0] _zz_37_;
  wire `ShiftCtrlEnum_defaultEncoding_type memory_SHIFT_CTRL;
  wire `ShiftCtrlEnum_defaultEncoding_type _zz_38_;
  wire [31:0] _zz_39_;
  wire `ShiftCtrlEnum_defaultEncoding_type execute_SHIFT_CTRL;
  wire `ShiftCtrlEnum_defaultEncoding_type _zz_40_;
  wire  _zz_41_;
  wire [31:0] _zz_42_;
  wire [31:0] _zz_43_;
  wire  execute_SRC_LESS_UNSIGNED;
  wire  execute_SRC2_FORCE_ZERO;
  wire  execute_SRC_USE_SUB_LESS;
  wire [31:0] _zz_44_;
  wire `Src2CtrlEnum_defaultEncoding_type execute_SRC2_CTRL;
  wire `Src2CtrlEnum_defaultEncoding_type _zz_45_;
  wire [31:0] _zz_46_;
  wire `Src1CtrlEnum_defaultEncoding_type execute_SRC1_CTRL;
  wire `Src1CtrlEnum_defaultEncoding_type _zz_47_;
  wire [31:0] _zz_48_;
  wire  decode_SRC_USE_SUB_LESS;
  wire  decode_SRC_ADD_ZERO;
  wire  _zz_49_;
  wire [31:0] execute_SRC_ADD_SUB;
  wire  execute_SRC_LESS;
  wire `AluCtrlEnum_defaultEncoding_type execute_ALU_CTRL;
  wire `AluCtrlEnum_defaultEncoding_type _zz_50_;
  wire [31:0] _zz_51_;
  wire [31:0] execute_SRC2;
  wire [31:0] execute_SRC1;
  wire `AluBitwiseCtrlEnum_defaultEncoding_type execute_ALU_BITWISE_CTRL;
  wire `AluBitwiseCtrlEnum_defaultEncoding_type _zz_52_;
  wire [31:0] _zz_53_;
  wire  _zz_54_;
  reg  _zz_55_;
  wire [31:0] _zz_56_;
  wire [31:0] _zz_57_;
  wire [31:0] decode_INSTRUCTION_ANTICIPATED;
  reg  decode_REGFILE_WRITE_VALID;
  wire  decode_LEGAL_INSTRUCTION;
  wire  decode_INSTRUCTION_READY;
  wire  _zz_58_;
  wire  _zz_59_;
  wire  _zz_60_;
  wire  _zz_61_;
  wire `Src1CtrlEnum_defaultEncoding_type _zz_62_;
  wire  _zz_63_;
  wire `EnvCtrlEnum_defaultEncoding_type _zz_64_;
  wire `AluCtrlEnum_defaultEncoding_type _zz_65_;
  wire  _zz_66_;
  wire  _zz_67_;
  wire `Src2CtrlEnum_defaultEncoding_type _zz_68_;
  wire  _zz_69_;
  wire  _zz_70_;
  wire `ShiftCtrlEnum_defaultEncoding_type _zz_71_;
  wire `AluBitwiseCtrlEnum_defaultEncoding_type _zz_72_;
  wire  _zz_73_;
  wire  _zz_74_;
  wire `BranchCtrlEnum_defaultEncoding_type _zz_75_;
  wire  _zz_76_;
  wire  writeBack_MEMORY_STORE;
  reg [31:0] _zz_77_;
  wire  writeBack_MEMORY_ENABLE;
  wire [1:0] writeBack_MEMORY_ADDRESS_LOW;
  wire [31:0] writeBack_MEMORY_READ_DATA;
  wire  memory_MMU_FAULT;
  wire [31:0] memory_MMU_RSP_physicalAddress;
  wire  memory_MMU_RSP_isIoAccess;
  wire  memory_MMU_RSP_allowRead;
  wire  memory_MMU_RSP_allowWrite;
  wire  memory_MMU_RSP_allowExecute;
  wire  memory_MMU_RSP_exception;
  wire  memory_MMU_RSP_refilling;
  wire [31:0] memory_PC;
  wire  memory_ALIGNEMENT_FAULT;
  wire [31:0] memory_REGFILE_WRITE_DATA;
  wire  memory_MEMORY_STORE;
  wire  memory_MEMORY_ENABLE;
  wire [31:0] _zz_78_;
  wire [31:0] _zz_79_;
  wire  _zz_80_;
  wire  _zz_81_;
  wire  _zz_82_;
  wire  _zz_83_;
  wire  _zz_84_;
  wire  _zz_85_;
  wire  execute_MMU_FAULT;
  wire [31:0] execute_MMU_RSP_physicalAddress;
  wire  execute_MMU_RSP_isIoAccess;
  wire  execute_MMU_RSP_allowRead;
  wire  execute_MMU_RSP_allowWrite;
  wire  execute_MMU_RSP_allowExecute;
  wire  execute_MMU_RSP_exception;
  wire  execute_MMU_RSP_refilling;
  wire  _zz_86_;
  wire [31:0] execute_SRC_ADD;
  wire [1:0] _zz_87_;
  wire [31:0] execute_RS2;
  wire [31:0] execute_INSTRUCTION;
  wire  execute_MEMORY_STORE;
  wire  execute_MEMORY_ENABLE;
  wire  execute_ALIGNEMENT_FAULT;
  wire  _zz_88_;
  wire  decode_MEMORY_ENABLE;
  reg [31:0] _zz_89_;
  reg [31:0] _zz_90_;
  wire [31:0] decode_PC;
  wire [31:0] _zz_91_;
  wire [31:0] _zz_92_;
  wire [31:0] _zz_93_;
  wire [31:0] decode_INSTRUCTION;
  wire [31:0] _zz_94_;
  wire [31:0] writeBack_PC;
  wire [31:0] writeBack_INSTRUCTION;
  reg  decode_arbitration_haltItself;
  reg  decode_arbitration_haltByOther;
  reg  decode_arbitration_removeIt;
  reg  decode_arbitration_flushIt;
  reg  decode_arbitration_flushNext;
  wire  decode_arbitration_isValid;
  wire  decode_arbitration_isStuck;
  wire  decode_arbitration_isStuckByOthers;
  wire  decode_arbitration_isFlushed;
  wire  decode_arbitration_isMoving;
  wire  decode_arbitration_isFiring;
  reg  execute_arbitration_haltItself;
  wire  execute_arbitration_haltByOther;
  reg  execute_arbitration_removeIt;
  wire  execute_arbitration_flushIt;
  reg  execute_arbitration_flushNext;
  reg  execute_arbitration_isValid;
  wire  execute_arbitration_isStuck;
  wire  execute_arbitration_isStuckByOthers;
  wire  execute_arbitration_isFlushed;
  wire  execute_arbitration_isMoving;
  wire  execute_arbitration_isFiring;
  reg  memory_arbitration_haltItself;
  wire  memory_arbitration_haltByOther;
  reg  memory_arbitration_removeIt;
  reg  memory_arbitration_flushIt;
  reg  memory_arbitration_flushNext;
  reg  memory_arbitration_isValid;
  wire  memory_arbitration_isStuck;
  wire  memory_arbitration_isStuckByOthers;
  wire  memory_arbitration_isFlushed;
  wire  memory_arbitration_isMoving;
  wire  memory_arbitration_isFiring;
  wire  writeBack_arbitration_haltItself;
  wire  writeBack_arbitration_haltByOther;
  reg  writeBack_arbitration_removeIt;
  wire  writeBack_arbitration_flushIt;
  reg  writeBack_arbitration_flushNext;
  reg  writeBack_arbitration_isValid;
  wire  writeBack_arbitration_isStuck;
  wire  writeBack_arbitration_isStuckByOthers;
  wire  writeBack_arbitration_isFlushed;
  wire  writeBack_arbitration_isMoving;
  wire  writeBack_arbitration_isFiring;
  wire [31:0] lastStageInstruction /* verilator public */ ;
  wire [31:0] lastStagePc /* verilator public */ ;
  wire  lastStageIsValid /* verilator public */ ;
  wire  lastStageIsFiring /* verilator public */ ;
  reg  IBusSimplePlugin_fetcherHalt;
  reg  IBusSimplePlugin_fetcherflushIt;
  reg  IBusSimplePlugin_incomingInstruction;
  wire  IBusSimplePlugin_pcValids_0;
  wire  IBusSimplePlugin_pcValids_1;
  wire  IBusSimplePlugin_pcValids_2;
  wire  IBusSimplePlugin_pcValids_3;
  wire  iBus_cmd_valid;
  wire  iBus_cmd_ready;
  wire [31:0] iBus_cmd_payload_pc;
  wire  iBus_rsp_valid;
  wire  iBus_rsp_payload_error;
  wire [31:0] iBus_rsp_payload_inst;
  wire  IBusSimplePlugin_decodeExceptionPort_valid;
  reg [3:0] IBusSimplePlugin_decodeExceptionPort_payload_code;
  wire [31:0] IBusSimplePlugin_decodeExceptionPort_payload_badAddr;
  wire  IBusSimplePlugin_mmuBus_cmd_isValid;
  wire [31:0] IBusSimplePlugin_mmuBus_cmd_virtualAddress;
  wire  IBusSimplePlugin_mmuBus_cmd_bypassTranslation;
  wire [31:0] IBusSimplePlugin_mmuBus_rsp_physicalAddress;
  wire  IBusSimplePlugin_mmuBus_rsp_isIoAccess;
  wire  IBusSimplePlugin_mmuBus_rsp_allowRead;
  wire  IBusSimplePlugin_mmuBus_rsp_allowWrite;
  wire  IBusSimplePlugin_mmuBus_rsp_allowExecute;
  wire  IBusSimplePlugin_mmuBus_rsp_exception;
  wire  IBusSimplePlugin_mmuBus_rsp_refilling;
  wire  IBusSimplePlugin_mmuBus_end;
  wire  IBusSimplePlugin_mmuBus_busy;
  wire  IBusSimplePlugin_redoBranch_valid;
  wire [31:0] IBusSimplePlugin_redoBranch_payload;
  reg  DBusSimplePlugin_memoryExceptionPort_valid;
  reg [3:0] DBusSimplePlugin_memoryExceptionPort_payload_code;
  wire [31:0] DBusSimplePlugin_memoryExceptionPort_payload_badAddr;
  wire  DBusSimplePlugin_mmuBus_cmd_isValid;
  wire [31:0] DBusSimplePlugin_mmuBus_cmd_virtualAddress;
  wire  DBusSimplePlugin_mmuBus_cmd_bypassTranslation;
  wire [31:0] DBusSimplePlugin_mmuBus_rsp_physicalAddress;
  wire  DBusSimplePlugin_mmuBus_rsp_isIoAccess;
  wire  DBusSimplePlugin_mmuBus_rsp_allowRead;
  wire  DBusSimplePlugin_mmuBus_rsp_allowWrite;
  wire  DBusSimplePlugin_mmuBus_rsp_allowExecute;
  wire  DBusSimplePlugin_mmuBus_rsp_exception;
  wire  DBusSimplePlugin_mmuBus_rsp_refilling;
  wire  DBusSimplePlugin_mmuBus_end;
  wire  DBusSimplePlugin_mmuBus_busy;
  reg  DBusSimplePlugin_redoBranch_valid;
  wire [31:0] DBusSimplePlugin_redoBranch_payload;
  wire  decodeExceptionPort_valid;
  wire [3:0] decodeExceptionPort_payload_code;
  wire [31:0] decodeExceptionPort_payload_badAddr;
  wire  BranchPlugin_jumpInterface_valid;
  wire [31:0] BranchPlugin_jumpInterface_payload;
  wire  BranchPlugin_branchExceptionPort_valid;
  wire [3:0] BranchPlugin_branchExceptionPort_payload_code;
  wire [31:0] BranchPlugin_branchExceptionPort_payload_badAddr;
  reg  CsrPlugin_jumpInterface_valid;
  reg [31:0] CsrPlugin_jumpInterface_payload;
  wire  CsrPlugin_exceptionPendings_0;
  wire  CsrPlugin_exceptionPendings_1;
  wire  CsrPlugin_exceptionPendings_2;
  wire  CsrPlugin_exceptionPendings_3;
  wire  externalInterrupt;
  wire  contextSwitching;
  reg [1:0] CsrPlugin_privilege;
  wire  CsrPlugin_forceMachineWire;
  reg  CsrPlugin_selfException_valid;
  reg [3:0] CsrPlugin_selfException_payload_code;
  wire [31:0] CsrPlugin_selfException_payload_badAddr;
  wire  CsrPlugin_allowInterrupts;
  wire  CsrPlugin_allowException;
  wire  IBusSimplePlugin_jump_pcLoad_valid;
  wire [31:0] IBusSimplePlugin_jump_pcLoad_payload;
  wire [3:0] _zz_95_;
  wire [3:0] _zz_96_;
  wire  _zz_97_;
  wire  _zz_98_;
  wire  _zz_99_;
  wire  IBusSimplePlugin_fetchPc_output_valid;
  wire  IBusSimplePlugin_fetchPc_output_ready;
  wire [31:0] IBusSimplePlugin_fetchPc_output_payload;
  reg [31:0] IBusSimplePlugin_fetchPc_pcReg /* verilator public */ ;
  reg  IBusSimplePlugin_fetchPc_corrected;
  reg  IBusSimplePlugin_fetchPc_pcRegPropagate;
  reg  IBusSimplePlugin_fetchPc_booted;
  reg  IBusSimplePlugin_fetchPc_inc;
  reg [31:0] IBusSimplePlugin_fetchPc_pc;
  reg  IBusSimplePlugin_iBusRsp_stages_0_input_valid;
  reg  IBusSimplePlugin_iBusRsp_stages_0_input_ready;
  wire [31:0] IBusSimplePlugin_iBusRsp_stages_0_input_payload;
  wire  IBusSimplePlugin_iBusRsp_stages_0_output_valid;
  wire  IBusSimplePlugin_iBusRsp_stages_0_output_ready;
  wire [31:0] IBusSimplePlugin_iBusRsp_stages_0_output_payload;
  reg  IBusSimplePlugin_iBusRsp_stages_0_halt;
  wire  IBusSimplePlugin_iBusRsp_stages_0_inputSample;
  wire  IBusSimplePlugin_iBusRsp_stages_1_input_valid;
  wire  IBusSimplePlugin_iBusRsp_stages_1_input_ready;
  wire [31:0] IBusSimplePlugin_iBusRsp_stages_1_input_payload;
  wire  IBusSimplePlugin_iBusRsp_stages_1_output_valid;
  wire  IBusSimplePlugin_iBusRsp_stages_1_output_ready;
  wire [31:0] IBusSimplePlugin_iBusRsp_stages_1_output_payload;
  wire  IBusSimplePlugin_iBusRsp_stages_1_halt;
  wire  IBusSimplePlugin_iBusRsp_stages_1_inputSample;
  wire  _zz_100_;
  wire  _zz_101_;
  wire  _zz_102_;
  wire  _zz_103_;
  reg  _zz_104_;
  reg  IBusSimplePlugin_iBusRsp_readyForError;
  wire  IBusSimplePlugin_iBusRsp_inputBeforeStage_valid;
  wire  IBusSimplePlugin_iBusRsp_inputBeforeStage_ready;
  wire [31:0] IBusSimplePlugin_iBusRsp_inputBeforeStage_payload_pc;
  wire  IBusSimplePlugin_iBusRsp_inputBeforeStage_payload_rsp_error;
  wire [31:0] IBusSimplePlugin_iBusRsp_inputBeforeStage_payload_rsp_inst;
  wire  IBusSimplePlugin_iBusRsp_inputBeforeStage_payload_isRvc;
  wire  IBusSimplePlugin_injector_decodeInput_valid;
  wire  IBusSimplePlugin_injector_decodeInput_ready;
  wire [31:0] IBusSimplePlugin_injector_decodeInput_payload_pc;
  wire  IBusSimplePlugin_injector_decodeInput_payload_rsp_error;
  wire [31:0] IBusSimplePlugin_injector_decodeInput_payload_rsp_inst;
  wire  IBusSimplePlugin_injector_decodeInput_payload_isRvc;
  reg  _zz_105_;
  reg [31:0] _zz_106_;
  reg  _zz_107_;
  reg [31:0] _zz_108_;
  reg  _zz_109_;
  reg  IBusSimplePlugin_injector_nextPcCalc_valids_0;
  reg  IBusSimplePlugin_injector_nextPcCalc_valids_1;
  reg  IBusSimplePlugin_injector_nextPcCalc_valids_2;
  reg  IBusSimplePlugin_injector_nextPcCalc_valids_3;
  reg  IBusSimplePlugin_injector_nextPcCalc_valids_4;
  reg  IBusSimplePlugin_injector_decodeRemoved;
  reg [31:0] IBusSimplePlugin_injector_formal_rawInDecode;
  reg  IBusSimplePlugin_cmd_valid;
  wire  IBusSimplePlugin_cmd_ready;
  wire [31:0] IBusSimplePlugin_cmd_payload_pc;
  reg [2:0] IBusSimplePlugin_pendingCmd;
  wire [2:0] IBusSimplePlugin_pendingCmdNext;
  reg [31:0] IBusSimplePlugin_mmu_joinCtx_physicalAddress;
  reg  IBusSimplePlugin_mmu_joinCtx_isIoAccess;
  reg  IBusSimplePlugin_mmu_joinCtx_allowRead;
  reg  IBusSimplePlugin_mmu_joinCtx_allowWrite;
  reg  IBusSimplePlugin_mmu_joinCtx_allowExecute;
  reg  IBusSimplePlugin_mmu_joinCtx_exception;
  reg  IBusSimplePlugin_mmu_joinCtx_refilling;
  reg [2:0] IBusSimplePlugin_rspJoin_discardCounter;
  wire  IBusSimplePlugin_rspJoin_rspBufferOutput_valid;
  wire  IBusSimplePlugin_rspJoin_rspBufferOutput_ready;
  wire  IBusSimplePlugin_rspJoin_rspBufferOutput_payload_error;
  wire [31:0] IBusSimplePlugin_rspJoin_rspBufferOutput_payload_inst;
  wire  iBus_rsp_takeWhen_valid;
  wire  iBus_rsp_takeWhen_payload_error;
  wire [31:0] iBus_rsp_takeWhen_payload_inst;
  wire [31:0] IBusSimplePlugin_rspJoin_fetchRsp_pc;
  reg  IBusSimplePlugin_rspJoin_fetchRsp_rsp_error;
  wire [31:0] IBusSimplePlugin_rspJoin_fetchRsp_rsp_inst;
  wire  IBusSimplePlugin_rspJoin_fetchRsp_isRvc;
  wire  IBusSimplePlugin_rspJoin_join_valid;
  wire  IBusSimplePlugin_rspJoin_join_ready;
  wire [31:0] IBusSimplePlugin_rspJoin_join_payload_pc;
  wire  IBusSimplePlugin_rspJoin_join_payload_rsp_error;
  wire [31:0] IBusSimplePlugin_rspJoin_join_payload_rsp_inst;
  wire  IBusSimplePlugin_rspJoin_join_payload_isRvc;
  reg  IBusSimplePlugin_rspJoin_exceptionDetected;
  reg  IBusSimplePlugin_rspJoin_redoRequired;
  wire  _zz_110_;
  wire  dBus_cmd_valid;
  wire  dBus_cmd_ready;
  wire  dBus_cmd_payload_wr;
  wire [31:0] dBus_cmd_payload_address;
  wire [31:0] dBus_cmd_payload_data;
  wire [1:0] dBus_cmd_payload_size;
  wire  dBus_rsp_ready;
  wire  dBus_rsp_error;
  wire [31:0] dBus_rsp_data;
  wire  _zz_111_;
  reg  execute_DBusSimplePlugin_skipCmd;
  reg [31:0] _zz_112_;
  reg [3:0] _zz_113_;
  wire [3:0] execute_DBusSimplePlugin_formalMask;
  reg [31:0] writeBack_DBusSimplePlugin_rspShifted;
  wire  _zz_114_;
  reg [31:0] _zz_115_;
  wire  _zz_116_;
  reg [31:0] _zz_117_;
  reg [31:0] writeBack_DBusSimplePlugin_rspFormated;
  wire [25:0] _zz_118_;
  wire  _zz_119_;
  wire  _zz_120_;
  wire  _zz_121_;
  wire `BranchCtrlEnum_defaultEncoding_type _zz_122_;
  wire `AluBitwiseCtrlEnum_defaultEncoding_type _zz_123_;
  wire `ShiftCtrlEnum_defaultEncoding_type _zz_124_;
  wire `Src2CtrlEnum_defaultEncoding_type _zz_125_;
  wire `AluCtrlEnum_defaultEncoding_type _zz_126_;
  wire `EnvCtrlEnum_defaultEncoding_type _zz_127_;
  wire `Src1CtrlEnum_defaultEncoding_type _zz_128_;
  wire [4:0] decode_RegFilePlugin_regFileReadAddress1;
  wire [4:0] decode_RegFilePlugin_regFileReadAddress2;
  wire [31:0] decode_RegFilePlugin_rs1Data;
  wire [31:0] decode_RegFilePlugin_rs2Data;
  reg  lastStageRegFileWrite_valid /* verilator public */ ;
  wire [4:0] lastStageRegFileWrite_payload_address /* verilator public */ ;
  wire [31:0] lastStageRegFileWrite_payload_data /* verilator public */ ;
  reg  _zz_129_;
  reg [31:0] execute_IntAluPlugin_bitwise;
  reg [31:0] _zz_130_;
  reg [31:0] _zz_131_;
  wire  _zz_132_;
  reg [19:0] _zz_133_;
  wire  _zz_134_;
  reg [19:0] _zz_135_;
  reg [31:0] _zz_136_;
  reg [31:0] execute_SrcPlugin_addSub;
  wire  execute_SrcPlugin_less;
  wire [4:0] execute_FullBarrelShifterPlugin_amplitude;
  reg [31:0] _zz_137_;
  wire [31:0] execute_FullBarrelShifterPlugin_reversed;
  reg [31:0] _zz_138_;
  reg  _zz_139_;
  reg  _zz_140_;
  wire  _zz_141_;
  reg  _zz_142_;
  reg [4:0] _zz_143_;
  wire  execute_BranchPlugin_eq;
  wire [2:0] _zz_144_;
  reg  _zz_145_;
  reg  _zz_146_;
  wire [31:0] execute_BranchPlugin_branch_src1;
  wire  _zz_147_;
  reg [10:0] _zz_148_;
  wire  _zz_149_;
  reg [19:0] _zz_150_;
  wire  _zz_151_;
  reg [18:0] _zz_152_;
  reg [31:0] _zz_153_;
  wire [31:0] execute_BranchPlugin_branch_src2;
  wire [31:0] execute_BranchPlugin_branchAdder;
  wire [1:0] CsrPlugin_misa_base;
  wire [25:0] CsrPlugin_misa_extensions;
  reg [1:0] CsrPlugin_mtvec_mode;
  reg [29:0] CsrPlugin_mtvec_base;
  reg [31:0] CsrPlugin_mepc;
  reg  CsrPlugin_mstatus_MIE;
  reg  CsrPlugin_mstatus_MPIE;
  reg [1:0] CsrPlugin_mstatus_MPP;
  reg  CsrPlugin_mip_MEIP;
  reg  CsrPlugin_mip_MTIP;
  reg  CsrPlugin_mip_MSIP;
  reg  CsrPlugin_mie_MEIE;
  reg  CsrPlugin_mie_MTIE;
  reg  CsrPlugin_mie_MSIE;
  reg  CsrPlugin_mcause_interrupt;
  reg [3:0] CsrPlugin_mcause_exceptionCode;
  reg [31:0] CsrPlugin_mtval;
  reg [63:0] CsrPlugin_mcycle = 64'b0000000000000000000000000000000000000000000000000000000000000000;
  reg [63:0] CsrPlugin_minstret = 64'b0000000000000000000000000000000000000000000000000000000000000000;
  wire  _zz_154_;
  wire  _zz_155_;
  wire  _zz_156_;
  reg  CsrPlugin_exceptionPortCtrl_exceptionValids_decode;
  reg  CsrPlugin_exceptionPortCtrl_exceptionValids_execute;
  reg  CsrPlugin_exceptionPortCtrl_exceptionValids_memory;
  reg  CsrPlugin_exceptionPortCtrl_exceptionValids_writeBack;
  reg  CsrPlugin_exceptionPortCtrl_exceptionValidsRegs_decode;
  reg  CsrPlugin_exceptionPortCtrl_exceptionValidsRegs_execute;
  reg  CsrPlugin_exceptionPortCtrl_exceptionValidsRegs_memory;
  reg  CsrPlugin_exceptionPortCtrl_exceptionValidsRegs_writeBack;
  reg [3:0] CsrPlugin_exceptionPortCtrl_exceptionContext_code;
  reg [31:0] CsrPlugin_exceptionPortCtrl_exceptionContext_badAddr;
  wire [1:0] CsrPlugin_exceptionPortCtrl_exceptionTargetPrivilegeUncapped;
  wire [1:0] CsrPlugin_exceptionPortCtrl_exceptionTargetPrivilege;
  wire [1:0] _zz_157_;
  wire  _zz_158_;
  wire [1:0] _zz_159_;
  wire  _zz_160_;
  reg  CsrPlugin_interrupt_valid;
  reg [3:0] CsrPlugin_interrupt_code /* verilator public */ ;
  reg [1:0] CsrPlugin_interrupt_targetPrivilege;
  wire  CsrPlugin_exception;
  wire  CsrPlugin_lastStageWasWfi;
  reg  CsrPlugin_pipelineLiberator_done;
  wire  CsrPlugin_interruptJump /* verilator public */ ;
  reg  CsrPlugin_hadException;
  reg [1:0] CsrPlugin_targetPrivilege;
  reg [3:0] CsrPlugin_trapCause;
  reg [1:0] CsrPlugin_xtvec_mode;
  reg [29:0] CsrPlugin_xtvec_base;
  wire  execute_CsrPlugin_inWfi /* verilator public */ ;
  reg  execute_CsrPlugin_wfiWake;
  wire  execute_CsrPlugin_blockedBySideEffects;
  reg  execute_CsrPlugin_illegalAccess;
  reg  execute_CsrPlugin_illegalInstruction;
  reg [31:0] execute_CsrPlugin_readData;
  reg  execute_CsrPlugin_writeInstruction;
  reg  execute_CsrPlugin_readInstruction;
  wire  execute_CsrPlugin_writeEnable;
  wire  execute_CsrPlugin_readEnable;
  wire [31:0] execute_CsrPlugin_readToWriteData;
  reg [31:0] execute_CsrPlugin_writeData;
  wire [11:0] execute_CsrPlugin_csrAddress;
  reg [31:0] externalInterruptArray_regNext;
  reg [31:0] _zz_161_;
  wire [31:0] _zz_162_;
  reg  execute_to_memory_ALIGNEMENT_FAULT;
  reg `Src2CtrlEnum_defaultEncoding_type decode_to_execute_SRC2_CTRL;
  reg  decode_to_execute_MEMORY_ENABLE;
  reg  execute_to_memory_MEMORY_ENABLE;
  reg  memory_to_writeBack_MEMORY_ENABLE;
  reg `AluBitwiseCtrlEnum_defaultEncoding_type decode_to_execute_ALU_BITWISE_CTRL;
  reg  decode_to_execute_BYPASSABLE_MEMORY_STAGE;
  reg  execute_to_memory_BYPASSABLE_MEMORY_STAGE;
  reg [31:0] decode_to_execute_RS1;
  reg  decode_to_execute_SRC_USE_SUB_LESS;
  reg  execute_to_memory_BRANCH_DO;
  reg  decode_to_execute_MEMORY_STORE;
  reg  execute_to_memory_MEMORY_STORE;
  reg  memory_to_writeBack_MEMORY_STORE;
  reg [31:0] execute_to_memory_BRANCH_CALC;
  reg  decode_to_execute_REGFILE_WRITE_VALID;
  reg  execute_to_memory_REGFILE_WRITE_VALID;
  reg  memory_to_writeBack_REGFILE_WRITE_VALID;
  reg [31:0] decode_to_execute_RS2;
  reg  execute_to_memory_MMU_FAULT;
  reg [31:0] execute_to_memory_SHIFT_RIGHT;
  reg [31:0] execute_to_memory_REGFILE_WRITE_DATA;
  reg [31:0] memory_to_writeBack_REGFILE_WRITE_DATA;
  reg  decode_to_execute_SRC_LESS_UNSIGNED;
  reg  decode_to_execute_BYPASSABLE_EXECUTE_STAGE;
  reg [1:0] execute_to_memory_MEMORY_ADDRESS_LOW;
  reg [1:0] memory_to_writeBack_MEMORY_ADDRESS_LOW;
  reg  decode_to_execute_CSR_READ_OPCODE;
  reg `EnvCtrlEnum_defaultEncoding_type decode_to_execute_ENV_CTRL;
  reg `EnvCtrlEnum_defaultEncoding_type execute_to_memory_ENV_CTRL;
  reg `EnvCtrlEnum_defaultEncoding_type memory_to_writeBack_ENV_CTRL;
  reg [31:0] decode_to_execute_FORMAL_PC_NEXT;
  reg [31:0] execute_to_memory_FORMAL_PC_NEXT;
  reg [31:0] memory_to_writeBack_FORMAL_PC_NEXT;
  reg [31:0] decode_to_execute_PC;
  reg [31:0] execute_to_memory_PC;
  reg [31:0] memory_to_writeBack_PC;
  reg  decode_to_execute_IS_CSR;
  reg [31:0] execute_to_memory_MMU_RSP_physicalAddress;
  reg  execute_to_memory_MMU_RSP_isIoAccess;
  reg  execute_to_memory_MMU_RSP_allowRead;
  reg  execute_to_memory_MMU_RSP_allowWrite;
  reg  execute_to_memory_MMU_RSP_allowExecute;
  reg  execute_to_memory_MMU_RSP_exception;
  reg  execute_to_memory_MMU_RSP_refilling;
  reg `BranchCtrlEnum_defaultEncoding_type decode_to_execute_BRANCH_CTRL;
  reg `AluCtrlEnum_defaultEncoding_type decode_to_execute_ALU_CTRL;
  reg  decode_to_execute_SRC2_FORCE_ZERO;
  reg `ShiftCtrlEnum_defaultEncoding_type decode_to_execute_SHIFT_CTRL;
  reg `ShiftCtrlEnum_defaultEncoding_type execute_to_memory_SHIFT_CTRL;
  reg [31:0] memory_to_writeBack_MEMORY_READ_DATA;
  reg  decode_to_execute_CSR_WRITE_OPCODE;
  reg `Src1CtrlEnum_defaultEncoding_type decode_to_execute_SRC1_CTRL;
  reg [31:0] decode_to_execute_INSTRUCTION;
  reg [31:0] execute_to_memory_INSTRUCTION;
  reg [31:0] memory_to_writeBack_INSTRUCTION;
  wire  iBus_cmd_m2sPipe_valid;
  wire  iBus_cmd_m2sPipe_ready;
  wire [31:0] iBus_cmd_m2sPipe_payload_pc;
  reg  _zz_163_;
  reg [31:0] _zz_164_;
  wire  dBus_cmd_halfPipe_valid;
  wire  dBus_cmd_halfPipe_ready;
  wire  dBus_cmd_halfPipe_payload_wr;
  wire [31:0] dBus_cmd_halfPipe_payload_address;
  wire [31:0] dBus_cmd_halfPipe_payload_data;
  wire [1:0] dBus_cmd_halfPipe_payload_size;
  reg  dBus_cmd_halfPipe_regs_valid;
  reg  dBus_cmd_halfPipe_regs_ready;
  reg  dBus_cmd_halfPipe_regs_payload_wr;
  reg [31:0] dBus_cmd_halfPipe_regs_payload_address;
  reg [31:0] dBus_cmd_halfPipe_regs_payload_data;
  reg [1:0] dBus_cmd_halfPipe_regs_payload_size;
  reg [3:0] _zz_165_;
  `ifndef SYNTHESIS
  reg [95:0] decode_SRC1_CTRL_string;
  reg [95:0] _zz_1__string;
  reg [95:0] _zz_2__string;
  reg [95:0] _zz_3__string;
  reg [71:0] _zz_4__string;
  reg [71:0] _zz_5__string;
  reg [71:0] decode_SHIFT_CTRL_string;
  reg [71:0] _zz_6__string;
  reg [71:0] _zz_7__string;
  reg [71:0] _zz_8__string;
  reg [63:0] decode_ALU_CTRL_string;
  reg [63:0] _zz_9__string;
  reg [63:0] _zz_10__string;
  reg [63:0] _zz_11__string;
  reg [31:0] decode_BRANCH_CTRL_string;
  reg [31:0] _zz_12__string;
  reg [31:0] _zz_13__string;
  reg [31:0] _zz_14__string;
  reg [39:0] _zz_15__string;
  reg [39:0] _zz_16__string;
  reg [39:0] _zz_17__string;
  reg [39:0] _zz_18__string;
  reg [39:0] decode_ENV_CTRL_string;
  reg [39:0] _zz_19__string;
  reg [39:0] _zz_20__string;
  reg [39:0] _zz_21__string;
  reg [39:0] decode_ALU_BITWISE_CTRL_string;
  reg [39:0] _zz_22__string;
  reg [39:0] _zz_23__string;
  reg [39:0] _zz_24__string;
  reg [23:0] decode_SRC2_CTRL_string;
  reg [23:0] _zz_25__string;
  reg [23:0] _zz_26__string;
  reg [23:0] _zz_27__string;
  reg [39:0] memory_ENV_CTRL_string;
  reg [39:0] _zz_29__string;
  reg [39:0] execute_ENV_CTRL_string;
  reg [39:0] _zz_30__string;
  reg [39:0] writeBack_ENV_CTRL_string;
  reg [39:0] _zz_33__string;
  reg [31:0] execute_BRANCH_CTRL_string;
  reg [31:0] _zz_35__string;
  reg [71:0] memory_SHIFT_CTRL_string;
  reg [71:0] _zz_38__string;
  reg [71:0] execute_SHIFT_CTRL_string;
  reg [71:0] _zz_40__string;
  reg [23:0] execute_SRC2_CTRL_string;
  reg [23:0] _zz_45__string;
  reg [95:0] execute_SRC1_CTRL_string;
  reg [95:0] _zz_47__string;
  reg [63:0] execute_ALU_CTRL_string;
  reg [63:0] _zz_50__string;
  reg [39:0] execute_ALU_BITWISE_CTRL_string;
  reg [39:0] _zz_52__string;
  reg [95:0] _zz_62__string;
  reg [39:0] _zz_64__string;
  reg [63:0] _zz_65__string;
  reg [23:0] _zz_68__string;
  reg [71:0] _zz_71__string;
  reg [39:0] _zz_72__string;
  reg [31:0] _zz_75__string;
  reg [31:0] _zz_122__string;
  reg [39:0] _zz_123__string;
  reg [71:0] _zz_124__string;
  reg [23:0] _zz_125__string;
  reg [63:0] _zz_126__string;
  reg [39:0] _zz_127__string;
  reg [95:0] _zz_128__string;
  reg [23:0] decode_to_execute_SRC2_CTRL_string;
  reg [39:0] decode_to_execute_ALU_BITWISE_CTRL_string;
  reg [39:0] decode_to_execute_ENV_CTRL_string;
  reg [39:0] execute_to_memory_ENV_CTRL_string;
  reg [39:0] memory_to_writeBack_ENV_CTRL_string;
  reg [31:0] decode_to_execute_BRANCH_CTRL_string;
  reg [63:0] decode_to_execute_ALU_CTRL_string;
  reg [71:0] decode_to_execute_SHIFT_CTRL_string;
  reg [71:0] execute_to_memory_SHIFT_CTRL_string;
  reg [95:0] decode_to_execute_SRC1_CTRL_string;
  `endif

  (* ram_style = "block" *) reg [31:0] RegFilePlugin_regFile [0:31] /* verilator public */ ;
  assign _zz_169_ = (execute_arbitration_isValid && execute_IS_CSR);
  assign _zz_170_ = ({decodeExceptionPort_valid,IBusSimplePlugin_decodeExceptionPort_valid} != (2'b00));
  assign _zz_171_ = ({BranchPlugin_branchExceptionPort_valid,DBusSimplePlugin_memoryExceptionPort_valid} != (2'b00));
  assign _zz_172_ = (CsrPlugin_hadException || CsrPlugin_interruptJump);
  assign _zz_173_ = (writeBack_arbitration_isValid && (writeBack_ENV_CTRL == `EnvCtrlEnum_defaultEncoding_XRET));
  assign _zz_174_ = writeBack_INSTRUCTION[29 : 28];
  assign _zz_175_ = (IBusSimplePlugin_mmuBus_rsp_exception || IBusSimplePlugin_mmuBus_rsp_refilling);
  assign _zz_176_ = ((IBusSimplePlugin_iBusRsp_stages_1_input_valid && (! IBusSimplePlugin_mmu_joinCtx_refilling)) && (IBusSimplePlugin_mmu_joinCtx_exception || (! IBusSimplePlugin_mmu_joinCtx_allowExecute)));
  assign _zz_177_ = ((dBus_rsp_ready && dBus_rsp_error) && (! memory_MEMORY_STORE));
  assign _zz_178_ = (! ((memory_arbitration_isValid && memory_MEMORY_ENABLE) && (1'b1 || (! memory_arbitration_isStuckByOthers))));
  assign _zz_179_ = (writeBack_arbitration_isValid && writeBack_REGFILE_WRITE_VALID);
  assign _zz_180_ = (1'b1 || (! 1'b1));
  assign _zz_181_ = (memory_arbitration_isValid && memory_REGFILE_WRITE_VALID);
  assign _zz_182_ = (1'b1 || (! memory_BYPASSABLE_MEMORY_STAGE));
  assign _zz_183_ = (execute_arbitration_isValid && execute_REGFILE_WRITE_VALID);
  assign _zz_184_ = (1'b1 || (! execute_BYPASSABLE_EXECUTE_STAGE));
  assign _zz_185_ = (CsrPlugin_privilege < execute_CsrPlugin_csrAddress[9 : 8]);
  assign _zz_186_ = (execute_arbitration_isValid && (execute_ENV_CTRL == `EnvCtrlEnum_defaultEncoding_ECALL));
  assign _zz_187_ = (CsrPlugin_mstatus_MIE || (CsrPlugin_privilege < (2'b11)));
  assign _zz_188_ = ((_zz_154_ && 1'b1) && (! 1'b0));
  assign _zz_189_ = ((_zz_155_ && 1'b1) && (! 1'b0));
  assign _zz_190_ = ((_zz_156_ && 1'b1) && (! 1'b0));
  assign _zz_191_ = (! dBus_cmd_halfPipe_regs_valid);
  assign _zz_192_ = writeBack_INSTRUCTION[13 : 12];
  assign _zz_193_ = execute_INSTRUCTION[13];
  assign _zz_194_ = (_zz_95_ - (4'b0001));
  assign _zz_195_ = {IBusSimplePlugin_fetchPc_inc,(2'b00)};
  assign _zz_196_ = {29'd0, _zz_195_};
  assign _zz_197_ = (IBusSimplePlugin_pendingCmd + _zz_199_);
  assign _zz_198_ = (IBusSimplePlugin_cmd_valid && IBusSimplePlugin_cmd_ready);
  assign _zz_199_ = {2'd0, _zz_198_};
  assign _zz_200_ = iBus_rsp_valid;
  assign _zz_201_ = {2'd0, _zz_200_};
  assign _zz_202_ = (iBus_rsp_valid && (IBusSimplePlugin_rspJoin_discardCounter != (3'b000)));
  assign _zz_203_ = {2'd0, _zz_202_};
  assign _zz_204_ = iBus_rsp_valid;
  assign _zz_205_ = {2'd0, _zz_204_};
  assign _zz_206_ = (memory_MEMORY_STORE ? (3'b110) : (3'b100));
  assign _zz_207_ = _zz_118_[2 : 2];
  assign _zz_208_ = _zz_118_[3 : 3];
  assign _zz_209_ = _zz_118_[8 : 8];
  assign _zz_210_ = _zz_118_[9 : 9];
  assign _zz_211_ = _zz_118_[12 : 12];
  assign _zz_212_ = _zz_118_[13 : 13];
  assign _zz_213_ = _zz_118_[18 : 18];
  assign _zz_214_ = _zz_118_[22 : 22];
  assign _zz_215_ = _zz_118_[23 : 23];
  assign _zz_216_ = _zz_118_[24 : 24];
  assign _zz_217_ = _zz_118_[25 : 25];
  assign _zz_218_ = execute_SRC_LESS;
  assign _zz_219_ = (3'b100);
  assign _zz_220_ = execute_INSTRUCTION[19 : 15];
  assign _zz_221_ = execute_INSTRUCTION[31 : 20];
  assign _zz_222_ = {execute_INSTRUCTION[31 : 25],execute_INSTRUCTION[11 : 7]};
  assign _zz_223_ = ($signed(_zz_224_) + $signed(_zz_227_));
  assign _zz_224_ = ($signed(_zz_225_) + $signed(_zz_226_));
  assign _zz_225_ = execute_SRC1;
  assign _zz_226_ = (execute_SRC_USE_SUB_LESS ? (~ execute_SRC2) : execute_SRC2);
  assign _zz_227_ = (execute_SRC_USE_SUB_LESS ? _zz_228_ : _zz_229_);
  assign _zz_228_ = (32'b00000000000000000000000000000001);
  assign _zz_229_ = (32'b00000000000000000000000000000000);
  assign _zz_230_ = ($signed(_zz_232_) >>> execute_FullBarrelShifterPlugin_amplitude);
  assign _zz_231_ = _zz_230_[31 : 0];
  assign _zz_232_ = {((execute_SHIFT_CTRL == `ShiftCtrlEnum_defaultEncoding_SRA_1) && execute_FullBarrelShifterPlugin_reversed[31]),execute_FullBarrelShifterPlugin_reversed};
  assign _zz_233_ = {{{execute_INSTRUCTION[31],execute_INSTRUCTION[19 : 12]},execute_INSTRUCTION[20]},execute_INSTRUCTION[30 : 21]};
  assign _zz_234_ = execute_INSTRUCTION[31 : 20];
  assign _zz_235_ = {{{execute_INSTRUCTION[31],execute_INSTRUCTION[7]},execute_INSTRUCTION[30 : 25]},execute_INSTRUCTION[11 : 8]};
  assign _zz_236_ = (_zz_157_ & (~ _zz_237_));
  assign _zz_237_ = (_zz_157_ - (2'b01));
  assign _zz_238_ = (_zz_159_ & (~ _zz_239_));
  assign _zz_239_ = (_zz_159_ - (2'b01));
  assign _zz_240_ = execute_CsrPlugin_writeData[7 : 7];
  assign _zz_241_ = execute_CsrPlugin_writeData[3 : 3];
  assign _zz_242_ = execute_CsrPlugin_writeData[3 : 3];
  assign _zz_243_ = execute_CsrPlugin_writeData[11 : 11];
  assign _zz_244_ = execute_CsrPlugin_writeData[7 : 7];
  assign _zz_245_ = execute_CsrPlugin_writeData[3 : 3];
  assign _zz_246_ = ({3'd0,_zz_165_} <<< dBus_cmd_halfPipe_payload_address[1 : 0]);
  assign _zz_247_ = 1'b1;
  assign _zz_248_ = 1'b1;
  assign _zz_249_ = {_zz_99_,_zz_98_};
  assign _zz_250_ = ((decode_INSTRUCTION & (32'b00000000000000000010000001010000)) == (32'b00000000000000000010000000010000));
  assign _zz_251_ = ((decode_INSTRUCTION & (32'b00000000000000000001000001010000)) == (32'b00000000000000000000000000010000));
  assign _zz_252_ = (decode_INSTRUCTION & (32'b00000000000000000000000001100100));
  assign _zz_253_ = (32'b00000000000000000000000000100100);
  assign _zz_254_ = ((decode_INSTRUCTION & (32'b00000000000000000000000000100000)) == (32'b00000000000000000000000000100000));
  assign _zz_255_ = {(_zz_260_ == _zz_261_),{_zz_262_,_zz_263_}};
  assign _zz_256_ = (3'b000);
  assign _zz_257_ = ({_zz_264_,{_zz_265_,_zz_266_}} != (3'b000));
  assign _zz_258_ = ({_zz_267_,_zz_268_} != (2'b00));
  assign _zz_259_ = {(_zz_269_ != _zz_270_),{_zz_271_,{_zz_272_,_zz_273_}}};
  assign _zz_260_ = (decode_INSTRUCTION & (32'b00000000000000000000000001000100));
  assign _zz_261_ = (32'b00000000000000000000000001000000);
  assign _zz_262_ = ((decode_INSTRUCTION & _zz_274_) == (32'b00000000000000000010000000010000));
  assign _zz_263_ = ((decode_INSTRUCTION & _zz_275_) == (32'b01000000000000000000000000110000));
  assign _zz_264_ = ((decode_INSTRUCTION & _zz_276_) == (32'b00000000000000000000000001000000));
  assign _zz_265_ = (_zz_277_ == _zz_278_);
  assign _zz_266_ = (_zz_279_ == _zz_280_);
  assign _zz_267_ = (_zz_281_ == _zz_282_);
  assign _zz_268_ = _zz_121_;
  assign _zz_269_ = {_zz_283_,_zz_121_};
  assign _zz_270_ = (2'b00);
  assign _zz_271_ = ({_zz_284_,_zz_285_} != (2'b00));
  assign _zz_272_ = (_zz_286_ != _zz_287_);
  assign _zz_273_ = {_zz_288_,{_zz_289_,_zz_290_}};
  assign _zz_274_ = (32'b00000000000000000010000000010100);
  assign _zz_275_ = (32'b01000000000000000000000000110100);
  assign _zz_276_ = (32'b00000000000000000000000001010000);
  assign _zz_277_ = (decode_INSTRUCTION & (32'b00000000000000000000000000111000));
  assign _zz_278_ = (32'b00000000000000000000000000000000);
  assign _zz_279_ = (decode_INSTRUCTION & (32'b00000000010000000011000001000000));
  assign _zz_280_ = (32'b00000000000000000000000001000000);
  assign _zz_281_ = (decode_INSTRUCTION & (32'b00000000000000000000000000010100));
  assign _zz_282_ = (32'b00000000000000000000000000000100);
  assign _zz_283_ = ((decode_INSTRUCTION & (32'b00000000000000000000000001000100)) == (32'b00000000000000000000000000000100));
  assign _zz_284_ = ((decode_INSTRUCTION & _zz_291_) == (32'b00000000000000000001000001010000));
  assign _zz_285_ = ((decode_INSTRUCTION & _zz_292_) == (32'b00000000000000000010000001010000));
  assign _zz_286_ = ((decode_INSTRUCTION & _zz_293_) == (32'b00000000000000000000000001010000));
  assign _zz_287_ = (1'b0);
  assign _zz_288_ = ((_zz_294_ == _zz_295_) != (1'b0));
  assign _zz_289_ = (_zz_296_ != (1'b0));
  assign _zz_290_ = {(_zz_297_ != _zz_298_),{_zz_299_,{_zz_300_,_zz_301_}}};
  assign _zz_291_ = (32'b00000000000000000001000001010000);
  assign _zz_292_ = (32'b00000000000000000010000001010000);
  assign _zz_293_ = (32'b00010000000000000011000001010000);
  assign _zz_294_ = (decode_INSTRUCTION & (32'b00010000010000000011000001010000));
  assign _zz_295_ = (32'b00010000000000000000000001010000);
  assign _zz_296_ = ((decode_INSTRUCTION & (32'b00000000000000000100000000010100)) == (32'b00000000000000000100000000010000));
  assign _zz_297_ = ((decode_INSTRUCTION & (32'b00000000000000000110000000010100)) == (32'b00000000000000000010000000010000));
  assign _zz_298_ = (1'b0);
  assign _zz_299_ = ({(_zz_302_ == _zz_303_),(_zz_304_ == _zz_305_)} != (2'b00));
  assign _zz_300_ = ((_zz_306_ == _zz_307_) != (1'b0));
  assign _zz_301_ = {({_zz_308_,_zz_309_} != (2'b00)),{(_zz_310_ != _zz_311_),{_zz_312_,{_zz_313_,_zz_314_}}}};
  assign _zz_302_ = (decode_INSTRUCTION & (32'b00000000000000000010000000010000));
  assign _zz_303_ = (32'b00000000000000000010000000000000);
  assign _zz_304_ = (decode_INSTRUCTION & (32'b00000000000000000101000000000000));
  assign _zz_305_ = (32'b00000000000000000001000000000000);
  assign _zz_306_ = (decode_INSTRUCTION & (32'b00000000000000000000000000010000));
  assign _zz_307_ = (32'b00000000000000000000000000010000);
  assign _zz_308_ = _zz_120_;
  assign _zz_309_ = ((decode_INSTRUCTION & _zz_315_) == (32'b00000000000000000000000000100000));
  assign _zz_310_ = {_zz_120_,(_zz_316_ == _zz_317_)};
  assign _zz_311_ = (2'b00);
  assign _zz_312_ = ({_zz_318_,_zz_319_} != (2'b00));
  assign _zz_313_ = (_zz_320_ != (1'b0));
  assign _zz_314_ = {(_zz_321_ != _zz_322_),{_zz_323_,{_zz_324_,_zz_325_}}};
  assign _zz_315_ = (32'b00000000000000000000000001110000);
  assign _zz_316_ = (decode_INSTRUCTION & (32'b00000000000000000000000000100000));
  assign _zz_317_ = (32'b00000000000000000000000000000000);
  assign _zz_318_ = ((decode_INSTRUCTION & (32'b00000000000000000000000000110100)) == (32'b00000000000000000000000000100000));
  assign _zz_319_ = ((decode_INSTRUCTION & (32'b00000000000000000000000001100100)) == (32'b00000000000000000000000000100000));
  assign _zz_320_ = ((decode_INSTRUCTION & (32'b00000000000000000000000001011000)) == (32'b00000000000000000000000000000000));
  assign _zz_321_ = ((decode_INSTRUCTION & _zz_326_) == (32'b00000000000000000101000000010000));
  assign _zz_322_ = (1'b0);
  assign _zz_323_ = ({_zz_327_,_zz_328_} != (2'b00));
  assign _zz_324_ = (_zz_329_ != (1'b0));
  assign _zz_325_ = {(_zz_330_ != _zz_331_),{_zz_332_,{_zz_333_,_zz_334_}}};
  assign _zz_326_ = (32'b00000000000000000111000001010100);
  assign _zz_327_ = ((decode_INSTRUCTION & (32'b01000000000000000011000001010100)) == (32'b01000000000000000001000000010000));
  assign _zz_328_ = ((decode_INSTRUCTION & (32'b00000000000000000111000001010100)) == (32'b00000000000000000001000000010000));
  assign _zz_329_ = ((decode_INSTRUCTION & (32'b00000000000000000001000000000000)) == (32'b00000000000000000001000000000000));
  assign _zz_330_ = ((decode_INSTRUCTION & _zz_335_) == (32'b00000000000000000010000000000000));
  assign _zz_331_ = (1'b0);
  assign _zz_332_ = ({_zz_336_,{_zz_337_,_zz_338_}} != (4'b0000));
  assign _zz_333_ = ({_zz_339_,_zz_340_} != (6'b000000));
  assign _zz_334_ = {(_zz_341_ != _zz_342_),(_zz_343_ != _zz_344_)};
  assign _zz_335_ = (32'b00000000000000000011000000000000);
  assign _zz_336_ = ((decode_INSTRUCTION & (32'b00000000000000000000000001000100)) == (32'b00000000000000000000000000000000));
  assign _zz_337_ = ((decode_INSTRUCTION & _zz_345_) == (32'b00000000000000000000000000000000));
  assign _zz_338_ = {(_zz_346_ == _zz_347_),(_zz_348_ == _zz_349_)};
  assign _zz_339_ = _zz_119_;
  assign _zz_340_ = {(_zz_350_ == _zz_351_),{_zz_352_,{_zz_353_,_zz_354_}}};
  assign _zz_341_ = {_zz_119_,(_zz_355_ == _zz_356_)};
  assign _zz_342_ = (2'b00);
  assign _zz_343_ = ((decode_INSTRUCTION & _zz_357_) == (32'b00000000000000000000000001000000));
  assign _zz_344_ = (1'b0);
  assign _zz_345_ = (32'b00000000000000000000000000011000);
  assign _zz_346_ = (decode_INSTRUCTION & (32'b00000000000000000110000000000100));
  assign _zz_347_ = (32'b00000000000000000010000000000000);
  assign _zz_348_ = (decode_INSTRUCTION & (32'b00000000000000000101000000000100));
  assign _zz_349_ = (32'b00000000000000000001000000000000);
  assign _zz_350_ = (decode_INSTRUCTION & (32'b00000000000000000001000000010000));
  assign _zz_351_ = (32'b00000000000000000001000000010000);
  assign _zz_352_ = ((decode_INSTRUCTION & (32'b00000000000000000010000000010000)) == (32'b00000000000000000010000000010000));
  assign _zz_353_ = ((decode_INSTRUCTION & (32'b00000000000000000000000001010000)) == (32'b00000000000000000000000000010000));
  assign _zz_354_ = {((decode_INSTRUCTION & (32'b00000000000000000000000000001100)) == (32'b00000000000000000000000000000100)),((decode_INSTRUCTION & (32'b00000000000000000000000000101000)) == (32'b00000000000000000000000000000000))};
  assign _zz_355_ = (decode_INSTRUCTION & (32'b00000000000000000000000000011100));
  assign _zz_356_ = (32'b00000000000000000000000000000100);
  assign _zz_357_ = (32'b00000000000000000000000001011000);
  assign _zz_358_ = (32'b00000000000000000001000001111111);
  assign _zz_359_ = (decode_INSTRUCTION & (32'b00000000000000000010000001111111));
  assign _zz_360_ = (32'b00000000000000000010000001110011);
  assign _zz_361_ = ((decode_INSTRUCTION & (32'b00000000000000000100000001111111)) == (32'b00000000000000000100000001100011));
  assign _zz_362_ = ((decode_INSTRUCTION & (32'b00000000000000000010000001111111)) == (32'b00000000000000000010000000010011));
  assign _zz_363_ = {((decode_INSTRUCTION & (32'b00000000000000000110000000111111)) == (32'b00000000000000000000000000100011)),{((decode_INSTRUCTION & (32'b00000000000000000010000001111111)) == (32'b00000000000000000000000000000011)),{((decode_INSTRUCTION & _zz_364_) == (32'b00000000000000000000000000000011)),{(_zz_365_ == _zz_366_),{_zz_367_,{_zz_368_,_zz_369_}}}}}};
  assign _zz_364_ = (32'b00000000000000000101000001011111);
  assign _zz_365_ = (decode_INSTRUCTION & (32'b00000000000000000111000001111011));
  assign _zz_366_ = (32'b00000000000000000000000001100011);
  assign _zz_367_ = ((decode_INSTRUCTION & (32'b00000000000000000110000001111111)) == (32'b00000000000000000000000000001111));
  assign _zz_368_ = ((decode_INSTRUCTION & (32'b11111110000000000000000001111111)) == (32'b00000000000000000000000000110011));
  assign _zz_369_ = {((decode_INSTRUCTION & (32'b10111100000000000111000001111111)) == (32'b00000000000000000101000000010011)),{((decode_INSTRUCTION & (32'b11111100000000000011000001111111)) == (32'b00000000000000000001000000010011)),{((decode_INSTRUCTION & _zz_370_) == (32'b00000000000000000101000000110011)),{(_zz_371_ == _zz_372_),{_zz_373_,{_zz_374_,_zz_375_}}}}}};
  assign _zz_370_ = (32'b10111110000000000111000001111111);
  assign _zz_371_ = (decode_INSTRUCTION & (32'b10111110000000000111000001111111));
  assign _zz_372_ = (32'b00000000000000000000000000110011);
  assign _zz_373_ = ((decode_INSTRUCTION & (32'b11011111111111111111111111111111)) == (32'b00010000001000000000000001110011));
  assign _zz_374_ = ((decode_INSTRUCTION & (32'b11111111111111111111111111111111)) == (32'b00010000010100000000000001110011));
  assign _zz_375_ = ((decode_INSTRUCTION & (32'b11111111111111111111111111111111)) == (32'b00000000000000000000000001110011));
  always @ (posedge clk) begin
    if(_zz_55_) begin
      RegFilePlugin_regFile[lastStageRegFileWrite_payload_address] <= lastStageRegFileWrite_payload_data;
    end
  end

  always @ (posedge clk) begin
    if(_zz_247_) begin
      _zz_166_ <= RegFilePlugin_regFile[decode_RegFilePlugin_regFileReadAddress1];
    end
  end

  always @ (posedge clk) begin
    if(_zz_248_) begin
      _zz_167_ <= RegFilePlugin_regFile[decode_RegFilePlugin_regFileReadAddress2];
    end
  end

  StreamFifoLowLatency IBusSimplePlugin_rspJoin_rspBuffer_c ( 
    .io_push_valid(iBus_rsp_takeWhen_valid),
    .io_push_ready(IBusSimplePlugin_rspJoin_rspBuffer_c_io_push_ready),
    .io_push_payload_error(iBus_rsp_takeWhen_payload_error),
    .io_push_payload_inst(iBus_rsp_takeWhen_payload_inst),
    .io_pop_valid(IBusSimplePlugin_rspJoin_rspBuffer_c_io_pop_valid),
    .io_pop_ready(IBusSimplePlugin_rspJoin_rspBufferOutput_ready),
    .io_pop_payload_error(IBusSimplePlugin_rspJoin_rspBuffer_c_io_pop_payload_error),
    .io_pop_payload_inst(IBusSimplePlugin_rspJoin_rspBuffer_c_io_pop_payload_inst),
    .io_flush(IBusSimplePlugin_fetcherflushIt),
    .io_occupancy(IBusSimplePlugin_rspJoin_rspBuffer_c_io_occupancy),
    .clk(clk),
    .reset(reset) 
  );
  always @(*) begin
    case(_zz_249_)
      2'b00 : begin
        _zz_168_ = CsrPlugin_jumpInterface_payload;
      end
      2'b01 : begin
        _zz_168_ = DBusSimplePlugin_redoBranch_payload;
      end
      2'b10 : begin
        _zz_168_ = BranchPlugin_jumpInterface_payload;
      end
      default : begin
        _zz_168_ = IBusSimplePlugin_redoBranch_payload;
      end
    endcase
  end

  `ifndef SYNTHESIS
  always @(*) begin
    case(decode_SRC1_CTRL)
      `Src1CtrlEnum_defaultEncoding_RS : decode_SRC1_CTRL_string = "RS          ";
      `Src1CtrlEnum_defaultEncoding_IMU : decode_SRC1_CTRL_string = "IMU         ";
      `Src1CtrlEnum_defaultEncoding_PC_INCREMENT : decode_SRC1_CTRL_string = "PC_INCREMENT";
      `Src1CtrlEnum_defaultEncoding_URS1 : decode_SRC1_CTRL_string = "URS1        ";
      default : decode_SRC1_CTRL_string = "????????????";
    endcase
  end
  always @(*) begin
    case(_zz_1_)
      `Src1CtrlEnum_defaultEncoding_RS : _zz_1__string = "RS          ";
      `Src1CtrlEnum_defaultEncoding_IMU : _zz_1__string = "IMU         ";
      `Src1CtrlEnum_defaultEncoding_PC_INCREMENT : _zz_1__string = "PC_INCREMENT";
      `Src1CtrlEnum_defaultEncoding_URS1 : _zz_1__string = "URS1        ";
      default : _zz_1__string = "????????????";
    endcase
  end
  always @(*) begin
    case(_zz_2_)
      `Src1CtrlEnum_defaultEncoding_RS : _zz_2__string = "RS          ";
      `Src1CtrlEnum_defaultEncoding_IMU : _zz_2__string = "IMU         ";
      `Src1CtrlEnum_defaultEncoding_PC_INCREMENT : _zz_2__string = "PC_INCREMENT";
      `Src1CtrlEnum_defaultEncoding_URS1 : _zz_2__string = "URS1        ";
      default : _zz_2__string = "????????????";
    endcase
  end
  always @(*) begin
    case(_zz_3_)
      `Src1CtrlEnum_defaultEncoding_RS : _zz_3__string = "RS          ";
      `Src1CtrlEnum_defaultEncoding_IMU : _zz_3__string = "IMU         ";
      `Src1CtrlEnum_defaultEncoding_PC_INCREMENT : _zz_3__string = "PC_INCREMENT";
      `Src1CtrlEnum_defaultEncoding_URS1 : _zz_3__string = "URS1        ";
      default : _zz_3__string = "????????????";
    endcase
  end
  always @(*) begin
    case(_zz_4_)
      `ShiftCtrlEnum_defaultEncoding_DISABLE_1 : _zz_4__string = "DISABLE_1";
      `ShiftCtrlEnum_defaultEncoding_SLL_1 : _zz_4__string = "SLL_1    ";
      `ShiftCtrlEnum_defaultEncoding_SRL_1 : _zz_4__string = "SRL_1    ";
      `ShiftCtrlEnum_defaultEncoding_SRA_1 : _zz_4__string = "SRA_1    ";
      default : _zz_4__string = "?????????";
    endcase
  end
  always @(*) begin
    case(_zz_5_)
      `ShiftCtrlEnum_defaultEncoding_DISABLE_1 : _zz_5__string = "DISABLE_1";
      `ShiftCtrlEnum_defaultEncoding_SLL_1 : _zz_5__string = "SLL_1    ";
      `ShiftCtrlEnum_defaultEncoding_SRL_1 : _zz_5__string = "SRL_1    ";
      `ShiftCtrlEnum_defaultEncoding_SRA_1 : _zz_5__string = "SRA_1    ";
      default : _zz_5__string = "?????????";
    endcase
  end
  always @(*) begin
    case(decode_SHIFT_CTRL)
      `ShiftCtrlEnum_defaultEncoding_DISABLE_1 : decode_SHIFT_CTRL_string = "DISABLE_1";
      `ShiftCtrlEnum_defaultEncoding_SLL_1 : decode_SHIFT_CTRL_string = "SLL_1    ";
      `ShiftCtrlEnum_defaultEncoding_SRL_1 : decode_SHIFT_CTRL_string = "SRL_1    ";
      `ShiftCtrlEnum_defaultEncoding_SRA_1 : decode_SHIFT_CTRL_string = "SRA_1    ";
      default : decode_SHIFT_CTRL_string = "?????????";
    endcase
  end
  always @(*) begin
    case(_zz_6_)
      `ShiftCtrlEnum_defaultEncoding_DISABLE_1 : _zz_6__string = "DISABLE_1";
      `ShiftCtrlEnum_defaultEncoding_SLL_1 : _zz_6__string = "SLL_1    ";
      `ShiftCtrlEnum_defaultEncoding_SRL_1 : _zz_6__string = "SRL_1    ";
      `ShiftCtrlEnum_defaultEncoding_SRA_1 : _zz_6__string = "SRA_1    ";
      default : _zz_6__string = "?????????";
    endcase
  end
  always @(*) begin
    case(_zz_7_)
      `ShiftCtrlEnum_defaultEncoding_DISABLE_1 : _zz_7__string = "DISABLE_1";
      `ShiftCtrlEnum_defaultEncoding_SLL_1 : _zz_7__string = "SLL_1    ";
      `ShiftCtrlEnum_defaultEncoding_SRL_1 : _zz_7__string = "SRL_1    ";
      `ShiftCtrlEnum_defaultEncoding_SRA_1 : _zz_7__string = "SRA_1    ";
      default : _zz_7__string = "?????????";
    endcase
  end
  always @(*) begin
    case(_zz_8_)
      `ShiftCtrlEnum_defaultEncoding_DISABLE_1 : _zz_8__string = "DISABLE_1";
      `ShiftCtrlEnum_defaultEncoding_SLL_1 : _zz_8__string = "SLL_1    ";
      `ShiftCtrlEnum_defaultEncoding_SRL_1 : _zz_8__string = "SRL_1    ";
      `ShiftCtrlEnum_defaultEncoding_SRA_1 : _zz_8__string = "SRA_1    ";
      default : _zz_8__string = "?????????";
    endcase
  end
  always @(*) begin
    case(decode_ALU_CTRL)
      `AluCtrlEnum_defaultEncoding_ADD_SUB : decode_ALU_CTRL_string = "ADD_SUB ";
      `AluCtrlEnum_defaultEncoding_SLT_SLTU : decode_ALU_CTRL_string = "SLT_SLTU";
      `AluCtrlEnum_defaultEncoding_BITWISE : decode_ALU_CTRL_string = "BITWISE ";
      default : decode_ALU_CTRL_string = "????????";
    endcase
  end
  always @(*) begin
    case(_zz_9_)
      `AluCtrlEnum_defaultEncoding_ADD_SUB : _zz_9__string = "ADD_SUB ";
      `AluCtrlEnum_defaultEncoding_SLT_SLTU : _zz_9__string = "SLT_SLTU";
      `AluCtrlEnum_defaultEncoding_BITWISE : _zz_9__string = "BITWISE ";
      default : _zz_9__string = "????????";
    endcase
  end
  always @(*) begin
    case(_zz_10_)
      `AluCtrlEnum_defaultEncoding_ADD_SUB : _zz_10__string = "ADD_SUB ";
      `AluCtrlEnum_defaultEncoding_SLT_SLTU : _zz_10__string = "SLT_SLTU";
      `AluCtrlEnum_defaultEncoding_BITWISE : _zz_10__string = "BITWISE ";
      default : _zz_10__string = "????????";
    endcase
  end
  always @(*) begin
    case(_zz_11_)
      `AluCtrlEnum_defaultEncoding_ADD_SUB : _zz_11__string = "ADD_SUB ";
      `AluCtrlEnum_defaultEncoding_SLT_SLTU : _zz_11__string = "SLT_SLTU";
      `AluCtrlEnum_defaultEncoding_BITWISE : _zz_11__string = "BITWISE ";
      default : _zz_11__string = "????????";
    endcase
  end
  always @(*) begin
    case(decode_BRANCH_CTRL)
      `BranchCtrlEnum_defaultEncoding_INC : decode_BRANCH_CTRL_string = "INC ";
      `BranchCtrlEnum_defaultEncoding_B : decode_BRANCH_CTRL_string = "B   ";
      `BranchCtrlEnum_defaultEncoding_JAL : decode_BRANCH_CTRL_string = "JAL ";
      `BranchCtrlEnum_defaultEncoding_JALR : decode_BRANCH_CTRL_string = "JALR";
      default : decode_BRANCH_CTRL_string = "????";
    endcase
  end
  always @(*) begin
    case(_zz_12_)
      `BranchCtrlEnum_defaultEncoding_INC : _zz_12__string = "INC ";
      `BranchCtrlEnum_defaultEncoding_B : _zz_12__string = "B   ";
      `BranchCtrlEnum_defaultEncoding_JAL : _zz_12__string = "JAL ";
      `BranchCtrlEnum_defaultEncoding_JALR : _zz_12__string = "JALR";
      default : _zz_12__string = "????";
    endcase
  end
  always @(*) begin
    case(_zz_13_)
      `BranchCtrlEnum_defaultEncoding_INC : _zz_13__string = "INC ";
      `BranchCtrlEnum_defaultEncoding_B : _zz_13__string = "B   ";
      `BranchCtrlEnum_defaultEncoding_JAL : _zz_13__string = "JAL ";
      `BranchCtrlEnum_defaultEncoding_JALR : _zz_13__string = "JALR";
      default : _zz_13__string = "????";
    endcase
  end
  always @(*) begin
    case(_zz_14_)
      `BranchCtrlEnum_defaultEncoding_INC : _zz_14__string = "INC ";
      `BranchCtrlEnum_defaultEncoding_B : _zz_14__string = "B   ";
      `BranchCtrlEnum_defaultEncoding_JAL : _zz_14__string = "JAL ";
      `BranchCtrlEnum_defaultEncoding_JALR : _zz_14__string = "JALR";
      default : _zz_14__string = "????";
    endcase
  end
  always @(*) begin
    case(_zz_15_)
      `EnvCtrlEnum_defaultEncoding_NONE : _zz_15__string = "NONE ";
      `EnvCtrlEnum_defaultEncoding_XRET : _zz_15__string = "XRET ";
      `EnvCtrlEnum_defaultEncoding_ECALL : _zz_15__string = "ECALL";
      default : _zz_15__string = "?????";
    endcase
  end
  always @(*) begin
    case(_zz_16_)
      `EnvCtrlEnum_defaultEncoding_NONE : _zz_16__string = "NONE ";
      `EnvCtrlEnum_defaultEncoding_XRET : _zz_16__string = "XRET ";
      `EnvCtrlEnum_defaultEncoding_ECALL : _zz_16__string = "ECALL";
      default : _zz_16__string = "?????";
    endcase
  end
  always @(*) begin
    case(_zz_17_)
      `EnvCtrlEnum_defaultEncoding_NONE : _zz_17__string = "NONE ";
      `EnvCtrlEnum_defaultEncoding_XRET : _zz_17__string = "XRET ";
      `EnvCtrlEnum_defaultEncoding_ECALL : _zz_17__string = "ECALL";
      default : _zz_17__string = "?????";
    endcase
  end
  always @(*) begin
    case(_zz_18_)
      `EnvCtrlEnum_defaultEncoding_NONE : _zz_18__string = "NONE ";
      `EnvCtrlEnum_defaultEncoding_XRET : _zz_18__string = "XRET ";
      `EnvCtrlEnum_defaultEncoding_ECALL : _zz_18__string = "ECALL";
      default : _zz_18__string = "?????";
    endcase
  end
  always @(*) begin
    case(decode_ENV_CTRL)
      `EnvCtrlEnum_defaultEncoding_NONE : decode_ENV_CTRL_string = "NONE ";
      `EnvCtrlEnum_defaultEncoding_XRET : decode_ENV_CTRL_string = "XRET ";
      `EnvCtrlEnum_defaultEncoding_ECALL : decode_ENV_CTRL_string = "ECALL";
      default : decode_ENV_CTRL_string = "?????";
    endcase
  end
  always @(*) begin
    case(_zz_19_)
      `EnvCtrlEnum_defaultEncoding_NONE : _zz_19__string = "NONE ";
      `EnvCtrlEnum_defaultEncoding_XRET : _zz_19__string = "XRET ";
      `EnvCtrlEnum_defaultEncoding_ECALL : _zz_19__string = "ECALL";
      default : _zz_19__string = "?????";
    endcase
  end
  always @(*) begin
    case(_zz_20_)
      `EnvCtrlEnum_defaultEncoding_NONE : _zz_20__string = "NONE ";
      `EnvCtrlEnum_defaultEncoding_XRET : _zz_20__string = "XRET ";
      `EnvCtrlEnum_defaultEncoding_ECALL : _zz_20__string = "ECALL";
      default : _zz_20__string = "?????";
    endcase
  end
  always @(*) begin
    case(_zz_21_)
      `EnvCtrlEnum_defaultEncoding_NONE : _zz_21__string = "NONE ";
      `EnvCtrlEnum_defaultEncoding_XRET : _zz_21__string = "XRET ";
      `EnvCtrlEnum_defaultEncoding_ECALL : _zz_21__string = "ECALL";
      default : _zz_21__string = "?????";
    endcase
  end
  always @(*) begin
    case(decode_ALU_BITWISE_CTRL)
      `AluBitwiseCtrlEnum_defaultEncoding_XOR_1 : decode_ALU_BITWISE_CTRL_string = "XOR_1";
      `AluBitwiseCtrlEnum_defaultEncoding_OR_1 : decode_ALU_BITWISE_CTRL_string = "OR_1 ";
      `AluBitwiseCtrlEnum_defaultEncoding_AND_1 : decode_ALU_BITWISE_CTRL_string = "AND_1";
      default : decode_ALU_BITWISE_CTRL_string = "?????";
    endcase
  end
  always @(*) begin
    case(_zz_22_)
      `AluBitwiseCtrlEnum_defaultEncoding_XOR_1 : _zz_22__string = "XOR_1";
      `AluBitwiseCtrlEnum_defaultEncoding_OR_1 : _zz_22__string = "OR_1 ";
      `AluBitwiseCtrlEnum_defaultEncoding_AND_1 : _zz_22__string = "AND_1";
      default : _zz_22__string = "?????";
    endcase
  end
  always @(*) begin
    case(_zz_23_)
      `AluBitwiseCtrlEnum_defaultEncoding_XOR_1 : _zz_23__string = "XOR_1";
      `AluBitwiseCtrlEnum_defaultEncoding_OR_1 : _zz_23__string = "OR_1 ";
      `AluBitwiseCtrlEnum_defaultEncoding_AND_1 : _zz_23__string = "AND_1";
      default : _zz_23__string = "?????";
    endcase
  end
  always @(*) begin
    case(_zz_24_)
      `AluBitwiseCtrlEnum_defaultEncoding_XOR_1 : _zz_24__string = "XOR_1";
      `AluBitwiseCtrlEnum_defaultEncoding_OR_1 : _zz_24__string = "OR_1 ";
      `AluBitwiseCtrlEnum_defaultEncoding_AND_1 : _zz_24__string = "AND_1";
      default : _zz_24__string = "?????";
    endcase
  end
  always @(*) begin
    case(decode_SRC2_CTRL)
      `Src2CtrlEnum_defaultEncoding_RS : decode_SRC2_CTRL_string = "RS ";
      `Src2CtrlEnum_defaultEncoding_IMI : decode_SRC2_CTRL_string = "IMI";
      `Src2CtrlEnum_defaultEncoding_IMS : decode_SRC2_CTRL_string = "IMS";
      `Src2CtrlEnum_defaultEncoding_PC : decode_SRC2_CTRL_string = "PC ";
      default : decode_SRC2_CTRL_string = "???";
    endcase
  end
  always @(*) begin
    case(_zz_25_)
      `Src2CtrlEnum_defaultEncoding_RS : _zz_25__string = "RS ";
      `Src2CtrlEnum_defaultEncoding_IMI : _zz_25__string = "IMI";
      `Src2CtrlEnum_defaultEncoding_IMS : _zz_25__string = "IMS";
      `Src2CtrlEnum_defaultEncoding_PC : _zz_25__string = "PC ";
      default : _zz_25__string = "???";
    endcase
  end
  always @(*) begin
    case(_zz_26_)
      `Src2CtrlEnum_defaultEncoding_RS : _zz_26__string = "RS ";
      `Src2CtrlEnum_defaultEncoding_IMI : _zz_26__string = "IMI";
      `Src2CtrlEnum_defaultEncoding_IMS : _zz_26__string = "IMS";
      `Src2CtrlEnum_defaultEncoding_PC : _zz_26__string = "PC ";
      default : _zz_26__string = "???";
    endcase
  end
  always @(*) begin
    case(_zz_27_)
      `Src2CtrlEnum_defaultEncoding_RS : _zz_27__string = "RS ";
      `Src2CtrlEnum_defaultEncoding_IMI : _zz_27__string = "IMI";
      `Src2CtrlEnum_defaultEncoding_IMS : _zz_27__string = "IMS";
      `Src2CtrlEnum_defaultEncoding_PC : _zz_27__string = "PC ";
      default : _zz_27__string = "???";
    endcase
  end
  always @(*) begin
    case(memory_ENV_CTRL)
      `EnvCtrlEnum_defaultEncoding_NONE : memory_ENV_CTRL_string = "NONE ";
      `EnvCtrlEnum_defaultEncoding_XRET : memory_ENV_CTRL_string = "XRET ";
      `EnvCtrlEnum_defaultEncoding_ECALL : memory_ENV_CTRL_string = "ECALL";
      default : memory_ENV_CTRL_string = "?????";
    endcase
  end
  always @(*) begin
    case(_zz_29_)
      `EnvCtrlEnum_defaultEncoding_NONE : _zz_29__string = "NONE ";
      `EnvCtrlEnum_defaultEncoding_XRET : _zz_29__string = "XRET ";
      `EnvCtrlEnum_defaultEncoding_ECALL : _zz_29__string = "ECALL";
      default : _zz_29__string = "?????";
    endcase
  end
  always @(*) begin
    case(execute_ENV_CTRL)
      `EnvCtrlEnum_defaultEncoding_NONE : execute_ENV_CTRL_string = "NONE ";
      `EnvCtrlEnum_defaultEncoding_XRET : execute_ENV_CTRL_string = "XRET ";
      `EnvCtrlEnum_defaultEncoding_ECALL : execute_ENV_CTRL_string = "ECALL";
      default : execute_ENV_CTRL_string = "?????";
    endcase
  end
  always @(*) begin
    case(_zz_30_)
      `EnvCtrlEnum_defaultEncoding_NONE : _zz_30__string = "NONE ";
      `EnvCtrlEnum_defaultEncoding_XRET : _zz_30__string = "XRET ";
      `EnvCtrlEnum_defaultEncoding_ECALL : _zz_30__string = "ECALL";
      default : _zz_30__string = "?????";
    endcase
  end
  always @(*) begin
    case(writeBack_ENV_CTRL)
      `EnvCtrlEnum_defaultEncoding_NONE : writeBack_ENV_CTRL_string = "NONE ";
      `EnvCtrlEnum_defaultEncoding_XRET : writeBack_ENV_CTRL_string = "XRET ";
      `EnvCtrlEnum_defaultEncoding_ECALL : writeBack_ENV_CTRL_string = "ECALL";
      default : writeBack_ENV_CTRL_string = "?????";
    endcase
  end
  always @(*) begin
    case(_zz_33_)
      `EnvCtrlEnum_defaultEncoding_NONE : _zz_33__string = "NONE ";
      `EnvCtrlEnum_defaultEncoding_XRET : _zz_33__string = "XRET ";
      `EnvCtrlEnum_defaultEncoding_ECALL : _zz_33__string = "ECALL";
      default : _zz_33__string = "?????";
    endcase
  end
  always @(*) begin
    case(execute_BRANCH_CTRL)
      `BranchCtrlEnum_defaultEncoding_INC : execute_BRANCH_CTRL_string = "INC ";
      `BranchCtrlEnum_defaultEncoding_B : execute_BRANCH_CTRL_string = "B   ";
      `BranchCtrlEnum_defaultEncoding_JAL : execute_BRANCH_CTRL_string = "JAL ";
      `BranchCtrlEnum_defaultEncoding_JALR : execute_BRANCH_CTRL_string = "JALR";
      default : execute_BRANCH_CTRL_string = "????";
    endcase
  end
  always @(*) begin
    case(_zz_35_)
      `BranchCtrlEnum_defaultEncoding_INC : _zz_35__string = "INC ";
      `BranchCtrlEnum_defaultEncoding_B : _zz_35__string = "B   ";
      `BranchCtrlEnum_defaultEncoding_JAL : _zz_35__string = "JAL ";
      `BranchCtrlEnum_defaultEncoding_JALR : _zz_35__string = "JALR";
      default : _zz_35__string = "????";
    endcase
  end
  always @(*) begin
    case(memory_SHIFT_CTRL)
      `ShiftCtrlEnum_defaultEncoding_DISABLE_1 : memory_SHIFT_CTRL_string = "DISABLE_1";
      `ShiftCtrlEnum_defaultEncoding_SLL_1 : memory_SHIFT_CTRL_string = "SLL_1    ";
      `ShiftCtrlEnum_defaultEncoding_SRL_1 : memory_SHIFT_CTRL_string = "SRL_1    ";
      `ShiftCtrlEnum_defaultEncoding_SRA_1 : memory_SHIFT_CTRL_string = "SRA_1    ";
      default : memory_SHIFT_CTRL_string = "?????????";
    endcase
  end
  always @(*) begin
    case(_zz_38_)
      `ShiftCtrlEnum_defaultEncoding_DISABLE_1 : _zz_38__string = "DISABLE_1";
      `ShiftCtrlEnum_defaultEncoding_SLL_1 : _zz_38__string = "SLL_1    ";
      `ShiftCtrlEnum_defaultEncoding_SRL_1 : _zz_38__string = "SRL_1    ";
      `ShiftCtrlEnum_defaultEncoding_SRA_1 : _zz_38__string = "SRA_1    ";
      default : _zz_38__string = "?????????";
    endcase
  end
  always @(*) begin
    case(execute_SHIFT_CTRL)
      `ShiftCtrlEnum_defaultEncoding_DISABLE_1 : execute_SHIFT_CTRL_string = "DISABLE_1";
      `ShiftCtrlEnum_defaultEncoding_SLL_1 : execute_SHIFT_CTRL_string = "SLL_1    ";
      `ShiftCtrlEnum_defaultEncoding_SRL_1 : execute_SHIFT_CTRL_string = "SRL_1    ";
      `ShiftCtrlEnum_defaultEncoding_SRA_1 : execute_SHIFT_CTRL_string = "SRA_1    ";
      default : execute_SHIFT_CTRL_string = "?????????";
    endcase
  end
  always @(*) begin
    case(_zz_40_)
      `ShiftCtrlEnum_defaultEncoding_DISABLE_1 : _zz_40__string = "DISABLE_1";
      `ShiftCtrlEnum_defaultEncoding_SLL_1 : _zz_40__string = "SLL_1    ";
      `ShiftCtrlEnum_defaultEncoding_SRL_1 : _zz_40__string = "SRL_1    ";
      `ShiftCtrlEnum_defaultEncoding_SRA_1 : _zz_40__string = "SRA_1    ";
      default : _zz_40__string = "?????????";
    endcase
  end
  always @(*) begin
    case(execute_SRC2_CTRL)
      `Src2CtrlEnum_defaultEncoding_RS : execute_SRC2_CTRL_string = "RS ";
      `Src2CtrlEnum_defaultEncoding_IMI : execute_SRC2_CTRL_string = "IMI";
      `Src2CtrlEnum_defaultEncoding_IMS : execute_SRC2_CTRL_string = "IMS";
      `Src2CtrlEnum_defaultEncoding_PC : execute_SRC2_CTRL_string = "PC ";
      default : execute_SRC2_CTRL_string = "???";
    endcase
  end
  always @(*) begin
    case(_zz_45_)
      `Src2CtrlEnum_defaultEncoding_RS : _zz_45__string = "RS ";
      `Src2CtrlEnum_defaultEncoding_IMI : _zz_45__string = "IMI";
      `Src2CtrlEnum_defaultEncoding_IMS : _zz_45__string = "IMS";
      `Src2CtrlEnum_defaultEncoding_PC : _zz_45__string = "PC ";
      default : _zz_45__string = "???";
    endcase
  end
  always @(*) begin
    case(execute_SRC1_CTRL)
      `Src1CtrlEnum_defaultEncoding_RS : execute_SRC1_CTRL_string = "RS          ";
      `Src1CtrlEnum_defaultEncoding_IMU : execute_SRC1_CTRL_string = "IMU         ";
      `Src1CtrlEnum_defaultEncoding_PC_INCREMENT : execute_SRC1_CTRL_string = "PC_INCREMENT";
      `Src1CtrlEnum_defaultEncoding_URS1 : execute_SRC1_CTRL_string = "URS1        ";
      default : execute_SRC1_CTRL_string = "????????????";
    endcase
  end
  always @(*) begin
    case(_zz_47_)
      `Src1CtrlEnum_defaultEncoding_RS : _zz_47__string = "RS          ";
      `Src1CtrlEnum_defaultEncoding_IMU : _zz_47__string = "IMU         ";
      `Src1CtrlEnum_defaultEncoding_PC_INCREMENT : _zz_47__string = "PC_INCREMENT";
      `Src1CtrlEnum_defaultEncoding_URS1 : _zz_47__string = "URS1        ";
      default : _zz_47__string = "????????????";
    endcase
  end
  always @(*) begin
    case(execute_ALU_CTRL)
      `AluCtrlEnum_defaultEncoding_ADD_SUB : execute_ALU_CTRL_string = "ADD_SUB ";
      `AluCtrlEnum_defaultEncoding_SLT_SLTU : execute_ALU_CTRL_string = "SLT_SLTU";
      `AluCtrlEnum_defaultEncoding_BITWISE : execute_ALU_CTRL_string = "BITWISE ";
      default : execute_ALU_CTRL_string = "????????";
    endcase
  end
  always @(*) begin
    case(_zz_50_)
      `AluCtrlEnum_defaultEncoding_ADD_SUB : _zz_50__string = "ADD_SUB ";
      `AluCtrlEnum_defaultEncoding_SLT_SLTU : _zz_50__string = "SLT_SLTU";
      `AluCtrlEnum_defaultEncoding_BITWISE : _zz_50__string = "BITWISE ";
      default : _zz_50__string = "????????";
    endcase
  end
  always @(*) begin
    case(execute_ALU_BITWISE_CTRL)
      `AluBitwiseCtrlEnum_defaultEncoding_XOR_1 : execute_ALU_BITWISE_CTRL_string = "XOR_1";
      `AluBitwiseCtrlEnum_defaultEncoding_OR_1 : execute_ALU_BITWISE_CTRL_string = "OR_1 ";
      `AluBitwiseCtrlEnum_defaultEncoding_AND_1 : execute_ALU_BITWISE_CTRL_string = "AND_1";
      default : execute_ALU_BITWISE_CTRL_string = "?????";
    endcase
  end
  always @(*) begin
    case(_zz_52_)
      `AluBitwiseCtrlEnum_defaultEncoding_XOR_1 : _zz_52__string = "XOR_1";
      `AluBitwiseCtrlEnum_defaultEncoding_OR_1 : _zz_52__string = "OR_1 ";
      `AluBitwiseCtrlEnum_defaultEncoding_AND_1 : _zz_52__string = "AND_1";
      default : _zz_52__string = "?????";
    endcase
  end
  always @(*) begin
    case(_zz_62_)
      `Src1CtrlEnum_defaultEncoding_RS : _zz_62__string = "RS          ";
      `Src1CtrlEnum_defaultEncoding_IMU : _zz_62__string = "IMU         ";
      `Src1CtrlEnum_defaultEncoding_PC_INCREMENT : _zz_62__string = "PC_INCREMENT";
      `Src1CtrlEnum_defaultEncoding_URS1 : _zz_62__string = "URS1        ";
      default : _zz_62__string = "????????????";
    endcase
  end
  always @(*) begin
    case(_zz_64_)
      `EnvCtrlEnum_defaultEncoding_NONE : _zz_64__string = "NONE ";
      `EnvCtrlEnum_defaultEncoding_XRET : _zz_64__string = "XRET ";
      `EnvCtrlEnum_defaultEncoding_ECALL : _zz_64__string = "ECALL";
      default : _zz_64__string = "?????";
    endcase
  end
  always @(*) begin
    case(_zz_65_)
      `AluCtrlEnum_defaultEncoding_ADD_SUB : _zz_65__string = "ADD_SUB ";
      `AluCtrlEnum_defaultEncoding_SLT_SLTU : _zz_65__string = "SLT_SLTU";
      `AluCtrlEnum_defaultEncoding_BITWISE : _zz_65__string = "BITWISE ";
      default : _zz_65__string = "????????";
    endcase
  end
  always @(*) begin
    case(_zz_68_)
      `Src2CtrlEnum_defaultEncoding_RS : _zz_68__string = "RS ";
      `Src2CtrlEnum_defaultEncoding_IMI : _zz_68__string = "IMI";
      `Src2CtrlEnum_defaultEncoding_IMS : _zz_68__string = "IMS";
      `Src2CtrlEnum_defaultEncoding_PC : _zz_68__string = "PC ";
      default : _zz_68__string = "???";
    endcase
  end
  always @(*) begin
    case(_zz_71_)
      `ShiftCtrlEnum_defaultEncoding_DISABLE_1 : _zz_71__string = "DISABLE_1";
      `ShiftCtrlEnum_defaultEncoding_SLL_1 : _zz_71__string = "SLL_1    ";
      `ShiftCtrlEnum_defaultEncoding_SRL_1 : _zz_71__string = "SRL_1    ";
      `ShiftCtrlEnum_defaultEncoding_SRA_1 : _zz_71__string = "SRA_1    ";
      default : _zz_71__string = "?????????";
    endcase
  end
  always @(*) begin
    case(_zz_72_)
      `AluBitwiseCtrlEnum_defaultEncoding_XOR_1 : _zz_72__string = "XOR_1";
      `AluBitwiseCtrlEnum_defaultEncoding_OR_1 : _zz_72__string = "OR_1 ";
      `AluBitwiseCtrlEnum_defaultEncoding_AND_1 : _zz_72__string = "AND_1";
      default : _zz_72__string = "?????";
    endcase
  end
  always @(*) begin
    case(_zz_75_)
      `BranchCtrlEnum_defaultEncoding_INC : _zz_75__string = "INC ";
      `BranchCtrlEnum_defaultEncoding_B : _zz_75__string = "B   ";
      `BranchCtrlEnum_defaultEncoding_JAL : _zz_75__string = "JAL ";
      `BranchCtrlEnum_defaultEncoding_JALR : _zz_75__string = "JALR";
      default : _zz_75__string = "????";
    endcase
  end
  always @(*) begin
    case(_zz_122_)
      `BranchCtrlEnum_defaultEncoding_INC : _zz_122__string = "INC ";
      `BranchCtrlEnum_defaultEncoding_B : _zz_122__string = "B   ";
      `BranchCtrlEnum_defaultEncoding_JAL : _zz_122__string = "JAL ";
      `BranchCtrlEnum_defaultEncoding_JALR : _zz_122__string = "JALR";
      default : _zz_122__string = "????";
    endcase
  end
  always @(*) begin
    case(_zz_123_)
      `AluBitwiseCtrlEnum_defaultEncoding_XOR_1 : _zz_123__string = "XOR_1";
      `AluBitwiseCtrlEnum_defaultEncoding_OR_1 : _zz_123__string = "OR_1 ";
      `AluBitwiseCtrlEnum_defaultEncoding_AND_1 : _zz_123__string = "AND_1";
      default : _zz_123__string = "?????";
    endcase
  end
  always @(*) begin
    case(_zz_124_)
      `ShiftCtrlEnum_defaultEncoding_DISABLE_1 : _zz_124__string = "DISABLE_1";
      `ShiftCtrlEnum_defaultEncoding_SLL_1 : _zz_124__string = "SLL_1    ";
      `ShiftCtrlEnum_defaultEncoding_SRL_1 : _zz_124__string = "SRL_1    ";
      `ShiftCtrlEnum_defaultEncoding_SRA_1 : _zz_124__string = "SRA_1    ";
      default : _zz_124__string = "?????????";
    endcase
  end
  always @(*) begin
    case(_zz_125_)
      `Src2CtrlEnum_defaultEncoding_RS : _zz_125__string = "RS ";
      `Src2CtrlEnum_defaultEncoding_IMI : _zz_125__string = "IMI";
      `Src2CtrlEnum_defaultEncoding_IMS : _zz_125__string = "IMS";
      `Src2CtrlEnum_defaultEncoding_PC : _zz_125__string = "PC ";
      default : _zz_125__string = "???";
    endcase
  end
  always @(*) begin
    case(_zz_126_)
      `AluCtrlEnum_defaultEncoding_ADD_SUB : _zz_126__string = "ADD_SUB ";
      `AluCtrlEnum_defaultEncoding_SLT_SLTU : _zz_126__string = "SLT_SLTU";
      `AluCtrlEnum_defaultEncoding_BITWISE : _zz_126__string = "BITWISE ";
      default : _zz_126__string = "????????";
    endcase
  end
  always @(*) begin
    case(_zz_127_)
      `EnvCtrlEnum_defaultEncoding_NONE : _zz_127__string = "NONE ";
      `EnvCtrlEnum_defaultEncoding_XRET : _zz_127__string = "XRET ";
      `EnvCtrlEnum_defaultEncoding_ECALL : _zz_127__string = "ECALL";
      default : _zz_127__string = "?????";
    endcase
  end
  always @(*) begin
    case(_zz_128_)
      `Src1CtrlEnum_defaultEncoding_RS : _zz_128__string = "RS          ";
      `Src1CtrlEnum_defaultEncoding_IMU : _zz_128__string = "IMU         ";
      `Src1CtrlEnum_defaultEncoding_PC_INCREMENT : _zz_128__string = "PC_INCREMENT";
      `Src1CtrlEnum_defaultEncoding_URS1 : _zz_128__string = "URS1        ";
      default : _zz_128__string = "????????????";
    endcase
  end
  always @(*) begin
    case(decode_to_execute_SRC2_CTRL)
      `Src2CtrlEnum_defaultEncoding_RS : decode_to_execute_SRC2_CTRL_string = "RS ";
      `Src2CtrlEnum_defaultEncoding_IMI : decode_to_execute_SRC2_CTRL_string = "IMI";
      `Src2CtrlEnum_defaultEncoding_IMS : decode_to_execute_SRC2_CTRL_string = "IMS";
      `Src2CtrlEnum_defaultEncoding_PC : decode_to_execute_SRC2_CTRL_string = "PC ";
      default : decode_to_execute_SRC2_CTRL_string = "???";
    endcase
  end
  always @(*) begin
    case(decode_to_execute_ALU_BITWISE_CTRL)
      `AluBitwiseCtrlEnum_defaultEncoding_XOR_1 : decode_to_execute_ALU_BITWISE_CTRL_string = "XOR_1";
      `AluBitwiseCtrlEnum_defaultEncoding_OR_1 : decode_to_execute_ALU_BITWISE_CTRL_string = "OR_1 ";
      `AluBitwiseCtrlEnum_defaultEncoding_AND_1 : decode_to_execute_ALU_BITWISE_CTRL_string = "AND_1";
      default : decode_to_execute_ALU_BITWISE_CTRL_string = "?????";
    endcase
  end
  always @(*) begin
    case(decode_to_execute_ENV_CTRL)
      `EnvCtrlEnum_defaultEncoding_NONE : decode_to_execute_ENV_CTRL_string = "NONE ";
      `EnvCtrlEnum_defaultEncoding_XRET : decode_to_execute_ENV_CTRL_string = "XRET ";
      `EnvCtrlEnum_defaultEncoding_ECALL : decode_to_execute_ENV_CTRL_string = "ECALL";
      default : decode_to_execute_ENV_CTRL_string = "?????";
    endcase
  end
  always @(*) begin
    case(execute_to_memory_ENV_CTRL)
      `EnvCtrlEnum_defaultEncoding_NONE : execute_to_memory_ENV_CTRL_string = "NONE ";
      `EnvCtrlEnum_defaultEncoding_XRET : execute_to_memory_ENV_CTRL_string = "XRET ";
      `EnvCtrlEnum_defaultEncoding_ECALL : execute_to_memory_ENV_CTRL_string = "ECALL";
      default : execute_to_memory_ENV_CTRL_string = "?????";
    endcase
  end
  always @(*) begin
    case(memory_to_writeBack_ENV_CTRL)
      `EnvCtrlEnum_defaultEncoding_NONE : memory_to_writeBack_ENV_CTRL_string = "NONE ";
      `EnvCtrlEnum_defaultEncoding_XRET : memory_to_writeBack_ENV_CTRL_string = "XRET ";
      `EnvCtrlEnum_defaultEncoding_ECALL : memory_to_writeBack_ENV_CTRL_string = "ECALL";
      default : memory_to_writeBack_ENV_CTRL_string = "?????";
    endcase
  end
  always @(*) begin
    case(decode_to_execute_BRANCH_CTRL)
      `BranchCtrlEnum_defaultEncoding_INC : decode_to_execute_BRANCH_CTRL_string = "INC ";
      `BranchCtrlEnum_defaultEncoding_B : decode_to_execute_BRANCH_CTRL_string = "B   ";
      `BranchCtrlEnum_defaultEncoding_JAL : decode_to_execute_BRANCH_CTRL_string = "JAL ";
      `BranchCtrlEnum_defaultEncoding_JALR : decode_to_execute_BRANCH_CTRL_string = "JALR";
      default : decode_to_execute_BRANCH_CTRL_string = "????";
    endcase
  end
  always @(*) begin
    case(decode_to_execute_ALU_CTRL)
      `AluCtrlEnum_defaultEncoding_ADD_SUB : decode_to_execute_ALU_CTRL_string = "ADD_SUB ";
      `AluCtrlEnum_defaultEncoding_SLT_SLTU : decode_to_execute_ALU_CTRL_string = "SLT_SLTU";
      `AluCtrlEnum_defaultEncoding_BITWISE : decode_to_execute_ALU_CTRL_string = "BITWISE ";
      default : decode_to_execute_ALU_CTRL_string = "????????";
    endcase
  end
  always @(*) begin
    case(decode_to_execute_SHIFT_CTRL)
      `ShiftCtrlEnum_defaultEncoding_DISABLE_1 : decode_to_execute_SHIFT_CTRL_string = "DISABLE_1";
      `ShiftCtrlEnum_defaultEncoding_SLL_1 : decode_to_execute_SHIFT_CTRL_string = "SLL_1    ";
      `ShiftCtrlEnum_defaultEncoding_SRL_1 : decode_to_execute_SHIFT_CTRL_string = "SRL_1    ";
      `ShiftCtrlEnum_defaultEncoding_SRA_1 : decode_to_execute_SHIFT_CTRL_string = "SRA_1    ";
      default : decode_to_execute_SHIFT_CTRL_string = "?????????";
    endcase
  end
  always @(*) begin
    case(execute_to_memory_SHIFT_CTRL)
      `ShiftCtrlEnum_defaultEncoding_DISABLE_1 : execute_to_memory_SHIFT_CTRL_string = "DISABLE_1";
      `ShiftCtrlEnum_defaultEncoding_SLL_1 : execute_to_memory_SHIFT_CTRL_string = "SLL_1    ";
      `ShiftCtrlEnum_defaultEncoding_SRL_1 : execute_to_memory_SHIFT_CTRL_string = "SRL_1    ";
      `ShiftCtrlEnum_defaultEncoding_SRA_1 : execute_to_memory_SHIFT_CTRL_string = "SRA_1    ";
      default : execute_to_memory_SHIFT_CTRL_string = "?????????";
    endcase
  end
  always @(*) begin
    case(decode_to_execute_SRC1_CTRL)
      `Src1CtrlEnum_defaultEncoding_RS : decode_to_execute_SRC1_CTRL_string = "RS          ";
      `Src1CtrlEnum_defaultEncoding_IMU : decode_to_execute_SRC1_CTRL_string = "IMU         ";
      `Src1CtrlEnum_defaultEncoding_PC_INCREMENT : decode_to_execute_SRC1_CTRL_string = "PC_INCREMENT";
      `Src1CtrlEnum_defaultEncoding_URS1 : decode_to_execute_SRC1_CTRL_string = "URS1        ";
      default : decode_to_execute_SRC1_CTRL_string = "????????????";
    endcase
  end
  `endif

  assign decode_SRC1_CTRL = _zz_1_;
  assign _zz_2_ = _zz_3_;
  assign decode_CSR_WRITE_OPCODE = _zz_32_;
  assign memory_MEMORY_READ_DATA = _zz_78_;
  assign _zz_4_ = _zz_5_;
  assign decode_SHIFT_CTRL = _zz_6_;
  assign _zz_7_ = _zz_8_;
  assign decode_SRC2_FORCE_ZERO = _zz_49_;
  assign decode_ALU_CTRL = _zz_9_;
  assign _zz_10_ = _zz_11_;
  assign decode_BRANCH_CTRL = _zz_12_;
  assign _zz_13_ = _zz_14_;
  assign decode_IS_CSR = _zz_63_;
  assign writeBack_FORMAL_PC_NEXT = memory_to_writeBack_FORMAL_PC_NEXT;
  assign memory_FORMAL_PC_NEXT = execute_to_memory_FORMAL_PC_NEXT;
  assign execute_FORMAL_PC_NEXT = decode_to_execute_FORMAL_PC_NEXT;
  assign decode_FORMAL_PC_NEXT = _zz_91_;
  assign _zz_15_ = _zz_16_;
  assign _zz_17_ = _zz_18_;
  assign decode_ENV_CTRL = _zz_19_;
  assign _zz_20_ = _zz_21_;
  assign decode_CSR_READ_OPCODE = _zz_31_;
  assign memory_MEMORY_ADDRESS_LOW = execute_to_memory_MEMORY_ADDRESS_LOW;
  assign execute_MEMORY_ADDRESS_LOW = _zz_87_;
  assign decode_BYPASSABLE_EXECUTE_STAGE = _zz_58_;
  assign decode_SRC_LESS_UNSIGNED = _zz_66_;
  assign writeBack_REGFILE_WRITE_DATA = memory_to_writeBack_REGFILE_WRITE_DATA;
  assign execute_REGFILE_WRITE_DATA = _zz_51_;
  assign execute_SHIFT_RIGHT = _zz_39_;
  assign decode_RS2 = _zz_56_;
  assign execute_BRANCH_CALC = _zz_34_;
  assign decode_MEMORY_STORE = _zz_60_;
  assign execute_BRANCH_DO = _zz_36_;
  assign decode_RS1 = _zz_57_;
  assign execute_BYPASSABLE_MEMORY_STAGE = decode_to_execute_BYPASSABLE_MEMORY_STAGE;
  assign decode_BYPASSABLE_MEMORY_STAGE = _zz_67_;
  assign decode_ALU_BITWISE_CTRL = _zz_22_;
  assign _zz_23_ = _zz_24_;
  assign decode_SRC2_CTRL = _zz_25_;
  assign _zz_26_ = _zz_27_;
  always @ (*) begin
    _zz_28_ = execute_REGFILE_WRITE_DATA;
    if(_zz_169_)begin
      _zz_28_ = execute_CsrPlugin_readData;
    end
  end

  assign execute_CSR_READ_OPCODE = decode_to_execute_CSR_READ_OPCODE;
  assign execute_CSR_WRITE_OPCODE = decode_to_execute_CSR_WRITE_OPCODE;
  assign execute_IS_CSR = decode_to_execute_IS_CSR;
  assign memory_ENV_CTRL = _zz_29_;
  assign execute_ENV_CTRL = _zz_30_;
  assign writeBack_ENV_CTRL = _zz_33_;
  assign memory_BRANCH_CALC = execute_to_memory_BRANCH_CALC;
  assign memory_BRANCH_DO = execute_to_memory_BRANCH_DO;
  assign execute_PC = decode_to_execute_PC;
  assign execute_RS1 = decode_to_execute_RS1;
  assign execute_BRANCH_CTRL = _zz_35_;
  assign decode_RS2_USE = _zz_69_;
  assign decode_RS1_USE = _zz_73_;
  assign execute_REGFILE_WRITE_VALID = decode_to_execute_REGFILE_WRITE_VALID;
  assign execute_BYPASSABLE_EXECUTE_STAGE = decode_to_execute_BYPASSABLE_EXECUTE_STAGE;
  assign memory_REGFILE_WRITE_VALID = execute_to_memory_REGFILE_WRITE_VALID;
  assign memory_INSTRUCTION = execute_to_memory_INSTRUCTION;
  assign memory_BYPASSABLE_MEMORY_STAGE = execute_to_memory_BYPASSABLE_MEMORY_STAGE;
  assign writeBack_REGFILE_WRITE_VALID = memory_to_writeBack_REGFILE_WRITE_VALID;
  assign memory_SHIFT_RIGHT = execute_to_memory_SHIFT_RIGHT;
  always @ (*) begin
    _zz_37_ = memory_REGFILE_WRITE_DATA;
    if(memory_arbitration_isValid)begin
      case(memory_SHIFT_CTRL)
        `ShiftCtrlEnum_defaultEncoding_SLL_1 : begin
          _zz_37_ = _zz_138_;
        end
        `ShiftCtrlEnum_defaultEncoding_SRL_1, `ShiftCtrlEnum_defaultEncoding_SRA_1 : begin
          _zz_37_ = memory_SHIFT_RIGHT;
        end
        default : begin
        end
      endcase
    end
  end

  assign memory_SHIFT_CTRL = _zz_38_;
  assign execute_SHIFT_CTRL = _zz_40_;
  assign execute_SRC_LESS_UNSIGNED = decode_to_execute_SRC_LESS_UNSIGNED;
  assign execute_SRC2_FORCE_ZERO = decode_to_execute_SRC2_FORCE_ZERO;
  assign execute_SRC_USE_SUB_LESS = decode_to_execute_SRC_USE_SUB_LESS;
  assign _zz_44_ = execute_PC;
  assign execute_SRC2_CTRL = _zz_45_;
  assign execute_SRC1_CTRL = _zz_47_;
  assign decode_SRC_USE_SUB_LESS = _zz_61_;
  assign decode_SRC_ADD_ZERO = _zz_59_;
  assign execute_SRC_ADD_SUB = _zz_43_;
  assign execute_SRC_LESS = _zz_41_;
  assign execute_ALU_CTRL = _zz_50_;
  assign execute_SRC2 = _zz_46_;
  assign execute_SRC1 = _zz_48_;
  assign execute_ALU_BITWISE_CTRL = _zz_52_;
  assign _zz_53_ = writeBack_INSTRUCTION;
  assign _zz_54_ = writeBack_REGFILE_WRITE_VALID;
  always @ (*) begin
    _zz_55_ = 1'b0;
    if(lastStageRegFileWrite_valid)begin
      _zz_55_ = 1'b1;
    end
  end

  assign decode_INSTRUCTION_ANTICIPATED = _zz_94_;
  always @ (*) begin
    decode_REGFILE_WRITE_VALID = _zz_74_;
    if((decode_INSTRUCTION[11 : 7] == (5'b00000)))begin
      decode_REGFILE_WRITE_VALID = 1'b0;
    end
  end

  assign decode_LEGAL_INSTRUCTION = _zz_76_;
  assign decode_INSTRUCTION_READY = 1'b1;
  assign writeBack_MEMORY_STORE = memory_to_writeBack_MEMORY_STORE;
  always @ (*) begin
    _zz_77_ = writeBack_REGFILE_WRITE_DATA;
    if((writeBack_arbitration_isValid && writeBack_MEMORY_ENABLE))begin
      _zz_77_ = writeBack_DBusSimplePlugin_rspFormated;
    end
  end

  assign writeBack_MEMORY_ENABLE = memory_to_writeBack_MEMORY_ENABLE;
  assign writeBack_MEMORY_ADDRESS_LOW = memory_to_writeBack_MEMORY_ADDRESS_LOW;
  assign writeBack_MEMORY_READ_DATA = memory_to_writeBack_MEMORY_READ_DATA;
  assign memory_MMU_FAULT = execute_to_memory_MMU_FAULT;
  assign memory_MMU_RSP_physicalAddress = execute_to_memory_MMU_RSP_physicalAddress;
  assign memory_MMU_RSP_isIoAccess = execute_to_memory_MMU_RSP_isIoAccess;
  assign memory_MMU_RSP_allowRead = execute_to_memory_MMU_RSP_allowRead;
  assign memory_MMU_RSP_allowWrite = execute_to_memory_MMU_RSP_allowWrite;
  assign memory_MMU_RSP_allowExecute = execute_to_memory_MMU_RSP_allowExecute;
  assign memory_MMU_RSP_exception = execute_to_memory_MMU_RSP_exception;
  assign memory_MMU_RSP_refilling = execute_to_memory_MMU_RSP_refilling;
  assign memory_PC = execute_to_memory_PC;
  assign memory_ALIGNEMENT_FAULT = execute_to_memory_ALIGNEMENT_FAULT;
  assign memory_REGFILE_WRITE_DATA = execute_to_memory_REGFILE_WRITE_DATA;
  assign memory_MEMORY_STORE = execute_to_memory_MEMORY_STORE;
  assign memory_MEMORY_ENABLE = execute_to_memory_MEMORY_ENABLE;
  assign execute_MMU_FAULT = _zz_86_;
  assign execute_MMU_RSP_physicalAddress = _zz_79_;
  assign execute_MMU_RSP_isIoAccess = _zz_80_;
  assign execute_MMU_RSP_allowRead = _zz_81_;
  assign execute_MMU_RSP_allowWrite = _zz_82_;
  assign execute_MMU_RSP_allowExecute = _zz_83_;
  assign execute_MMU_RSP_exception = _zz_84_;
  assign execute_MMU_RSP_refilling = _zz_85_;
  assign execute_SRC_ADD = _zz_42_;
  assign execute_RS2 = decode_to_execute_RS2;
  assign execute_INSTRUCTION = decode_to_execute_INSTRUCTION;
  assign execute_MEMORY_STORE = decode_to_execute_MEMORY_STORE;
  assign execute_MEMORY_ENABLE = decode_to_execute_MEMORY_ENABLE;
  assign execute_ALIGNEMENT_FAULT = _zz_88_;
  assign decode_MEMORY_ENABLE = _zz_70_;
  always @ (*) begin
    _zz_89_ = memory_FORMAL_PC_NEXT;
    if(DBusSimplePlugin_redoBranch_valid)begin
      _zz_89_ = DBusSimplePlugin_redoBranch_payload;
    end
    if(BranchPlugin_jumpInterface_valid)begin
      _zz_89_ = BranchPlugin_jumpInterface_payload;
    end
  end

  always @ (*) begin
    _zz_90_ = decode_FORMAL_PC_NEXT;
    if(IBusSimplePlugin_redoBranch_valid)begin
      _zz_90_ = IBusSimplePlugin_redoBranch_payload;
    end
  end

  assign decode_PC = _zz_93_;
  assign decode_INSTRUCTION = _zz_92_;
  assign writeBack_PC = memory_to_writeBack_PC;
  assign writeBack_INSTRUCTION = memory_to_writeBack_INSTRUCTION;
  always @ (*) begin
    decode_arbitration_haltItself = 1'b0;
    if(((DBusSimplePlugin_mmuBus_busy && decode_arbitration_isValid) && decode_MEMORY_ENABLE))begin
      decode_arbitration_haltItself = 1'b1;
    end
  end

  always @ (*) begin
    decode_arbitration_haltByOther = 1'b0;
    if((decode_arbitration_isValid && (_zz_139_ || _zz_140_)))begin
      decode_arbitration_haltByOther = 1'b1;
    end
    if((CsrPlugin_interrupt_valid && CsrPlugin_allowInterrupts))begin
      decode_arbitration_haltByOther = decode_arbitration_isValid;
    end
    if(({(writeBack_arbitration_isValid && (writeBack_ENV_CTRL == `EnvCtrlEnum_defaultEncoding_XRET)),{(memory_arbitration_isValid && (memory_ENV_CTRL == `EnvCtrlEnum_defaultEncoding_XRET)),(execute_arbitration_isValid && (execute_ENV_CTRL == `EnvCtrlEnum_defaultEncoding_XRET))}} != (3'b000)))begin
      decode_arbitration_haltByOther = 1'b1;
    end
  end

  always @ (*) begin
    decode_arbitration_removeIt = 1'b0;
    if(_zz_170_)begin
      decode_arbitration_removeIt = 1'b1;
    end
    if(decode_arbitration_isFlushed)begin
      decode_arbitration_removeIt = 1'b1;
    end
  end

  always @ (*) begin
    decode_arbitration_flushIt = 1'b0;
    if(IBusSimplePlugin_redoBranch_valid)begin
      decode_arbitration_flushIt = 1'b1;
    end
  end

  always @ (*) begin
    decode_arbitration_flushNext = 1'b0;
    if(IBusSimplePlugin_redoBranch_valid)begin
      decode_arbitration_flushNext = 1'b1;
    end
    if(_zz_170_)begin
      decode_arbitration_flushNext = 1'b1;
    end
  end

  always @ (*) begin
    execute_arbitration_haltItself = 1'b0;
    if(((((execute_arbitration_isValid && execute_MEMORY_ENABLE) && (! dBus_cmd_ready)) && (! execute_DBusSimplePlugin_skipCmd)) && (! _zz_111_)))begin
      execute_arbitration_haltItself = 1'b1;
    end
    if(_zz_169_)begin
      if(execute_CsrPlugin_blockedBySideEffects)begin
        execute_arbitration_haltItself = 1'b1;
      end
    end
  end

  assign execute_arbitration_haltByOther = 1'b0;
  always @ (*) begin
    execute_arbitration_removeIt = 1'b0;
    if(CsrPlugin_selfException_valid)begin
      execute_arbitration_removeIt = 1'b1;
    end
    if(execute_arbitration_isFlushed)begin
      execute_arbitration_removeIt = 1'b1;
    end
  end

  assign execute_arbitration_flushIt = 1'b0;
  always @ (*) begin
    execute_arbitration_flushNext = 1'b0;
    if(CsrPlugin_selfException_valid)begin
      execute_arbitration_flushNext = 1'b1;
    end
  end

  always @ (*) begin
    memory_arbitration_haltItself = 1'b0;
    if((((memory_arbitration_isValid && memory_MEMORY_ENABLE) && (! memory_MEMORY_STORE)) && ((! dBus_rsp_ready) || 1'b0)))begin
      memory_arbitration_haltItself = 1'b1;
    end
  end

  assign memory_arbitration_haltByOther = 1'b0;
  always @ (*) begin
    memory_arbitration_removeIt = 1'b0;
    if(_zz_171_)begin
      memory_arbitration_removeIt = 1'b1;
    end
    if(memory_arbitration_isFlushed)begin
      memory_arbitration_removeIt = 1'b1;
    end
  end

  always @ (*) begin
    memory_arbitration_flushIt = 1'b0;
    if(DBusSimplePlugin_redoBranch_valid)begin
      memory_arbitration_flushIt = 1'b1;
    end
  end

  always @ (*) begin
    memory_arbitration_flushNext = 1'b0;
    if(DBusSimplePlugin_redoBranch_valid)begin
      memory_arbitration_flushNext = 1'b1;
    end
    if(BranchPlugin_jumpInterface_valid)begin
      memory_arbitration_flushNext = 1'b1;
    end
    if(_zz_171_)begin
      memory_arbitration_flushNext = 1'b1;
    end
  end

  assign writeBack_arbitration_haltItself = 1'b0;
  assign writeBack_arbitration_haltByOther = 1'b0;
  always @ (*) begin
    writeBack_arbitration_removeIt = 1'b0;
    if(writeBack_arbitration_isFlushed)begin
      writeBack_arbitration_removeIt = 1'b1;
    end
  end

  assign writeBack_arbitration_flushIt = 1'b0;
  always @ (*) begin
    writeBack_arbitration_flushNext = 1'b0;
    if(_zz_172_)begin
      writeBack_arbitration_flushNext = 1'b1;
    end
    if(_zz_173_)begin
      writeBack_arbitration_flushNext = 1'b1;
    end
  end

  assign lastStageInstruction = writeBack_INSTRUCTION;
  assign lastStagePc = writeBack_PC;
  assign lastStageIsValid = writeBack_arbitration_isValid;
  assign lastStageIsFiring = writeBack_arbitration_isFiring;
  always @ (*) begin
    IBusSimplePlugin_fetcherHalt = 1'b0;
    if(({CsrPlugin_exceptionPortCtrl_exceptionValids_writeBack,{CsrPlugin_exceptionPortCtrl_exceptionValids_memory,{CsrPlugin_exceptionPortCtrl_exceptionValids_execute,CsrPlugin_exceptionPortCtrl_exceptionValids_decode}}} != (4'b0000)))begin
      IBusSimplePlugin_fetcherHalt = 1'b1;
    end
    if(_zz_172_)begin
      IBusSimplePlugin_fetcherHalt = 1'b1;
    end
    if(_zz_173_)begin
      IBusSimplePlugin_fetcherHalt = 1'b1;
    end
  end

  always @ (*) begin
    IBusSimplePlugin_fetcherflushIt = 1'b0;
    if(({writeBack_arbitration_flushNext,{memory_arbitration_flushNext,{execute_arbitration_flushNext,decode_arbitration_flushNext}}} != (4'b0000)))begin
      IBusSimplePlugin_fetcherflushIt = 1'b1;
    end
  end

  always @ (*) begin
    IBusSimplePlugin_incomingInstruction = 1'b0;
    if(IBusSimplePlugin_iBusRsp_stages_1_input_valid)begin
      IBusSimplePlugin_incomingInstruction = 1'b1;
    end
    if(IBusSimplePlugin_injector_decodeInput_valid)begin
      IBusSimplePlugin_incomingInstruction = 1'b1;
    end
  end

  always @ (*) begin
    CsrPlugin_jumpInterface_valid = 1'b0;
    if(_zz_172_)begin
      CsrPlugin_jumpInterface_valid = 1'b1;
    end
    if(_zz_173_)begin
      CsrPlugin_jumpInterface_valid = 1'b1;
    end
  end

  always @ (*) begin
    CsrPlugin_jumpInterface_payload = (32'bxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxx);
    if(_zz_172_)begin
      CsrPlugin_jumpInterface_payload = {CsrPlugin_xtvec_base,(2'b00)};
    end
    if(_zz_173_)begin
      case(_zz_174_)
        2'b11 : begin
          CsrPlugin_jumpInterface_payload = CsrPlugin_mepc;
        end
        default : begin
        end
      endcase
    end
  end

  assign CsrPlugin_forceMachineWire = 1'b0;
  assign CsrPlugin_allowInterrupts = 1'b1;
  assign CsrPlugin_allowException = 1'b1;
  assign IBusSimplePlugin_jump_pcLoad_valid = ({CsrPlugin_jumpInterface_valid,{BranchPlugin_jumpInterface_valid,{DBusSimplePlugin_redoBranch_valid,IBusSimplePlugin_redoBranch_valid}}} != (4'b0000));
  assign _zz_95_ = {IBusSimplePlugin_redoBranch_valid,{BranchPlugin_jumpInterface_valid,{DBusSimplePlugin_redoBranch_valid,CsrPlugin_jumpInterface_valid}}};
  assign _zz_96_ = (_zz_95_ & (~ _zz_194_));
  assign _zz_97_ = _zz_96_[3];
  assign _zz_98_ = (_zz_96_[1] || _zz_97_);
  assign _zz_99_ = (_zz_96_[2] || _zz_97_);
  assign IBusSimplePlugin_jump_pcLoad_payload = _zz_168_;
  always @ (*) begin
    IBusSimplePlugin_fetchPc_corrected = 1'b0;
    if(IBusSimplePlugin_jump_pcLoad_valid)begin
      IBusSimplePlugin_fetchPc_corrected = 1'b1;
    end
  end

  always @ (*) begin
    IBusSimplePlugin_fetchPc_pcRegPropagate = 1'b0;
    if(IBusSimplePlugin_iBusRsp_stages_1_input_ready)begin
      IBusSimplePlugin_fetchPc_pcRegPropagate = 1'b1;
    end
  end

  always @ (*) begin
    IBusSimplePlugin_fetchPc_pc = (IBusSimplePlugin_fetchPc_pcReg + _zz_196_);
    if(IBusSimplePlugin_jump_pcLoad_valid)begin
      IBusSimplePlugin_fetchPc_pc = IBusSimplePlugin_jump_pcLoad_payload;
    end
    IBusSimplePlugin_fetchPc_pc[0] = 1'b0;
    IBusSimplePlugin_fetchPc_pc[1] = 1'b0;
  end

  assign IBusSimplePlugin_fetchPc_output_valid = ((! IBusSimplePlugin_fetcherHalt) && IBusSimplePlugin_fetchPc_booted);
  assign IBusSimplePlugin_fetchPc_output_payload = IBusSimplePlugin_fetchPc_pc;
  always @ (*) begin
    IBusSimplePlugin_iBusRsp_stages_0_input_valid = IBusSimplePlugin_fetchPc_output_valid;
    if(IBusSimplePlugin_mmuBus_busy)begin
      IBusSimplePlugin_iBusRsp_stages_0_input_valid = 1'b0;
    end
  end

  assign IBusSimplePlugin_fetchPc_output_ready = IBusSimplePlugin_iBusRsp_stages_0_input_ready;
  assign IBusSimplePlugin_iBusRsp_stages_0_input_payload = IBusSimplePlugin_fetchPc_output_payload;
  assign IBusSimplePlugin_iBusRsp_stages_0_inputSample = 1'b1;
  always @ (*) begin
    IBusSimplePlugin_iBusRsp_stages_0_halt = 1'b0;
    if((IBusSimplePlugin_iBusRsp_stages_0_input_valid && ((! IBusSimplePlugin_cmd_valid) || (! IBusSimplePlugin_cmd_ready))))begin
      IBusSimplePlugin_iBusRsp_stages_0_halt = 1'b1;
    end
    if(_zz_175_)begin
      IBusSimplePlugin_iBusRsp_stages_0_halt = 1'b0;
    end
  end

  assign _zz_100_ = (! IBusSimplePlugin_iBusRsp_stages_0_halt);
  always @ (*) begin
    IBusSimplePlugin_iBusRsp_stages_0_input_ready = (IBusSimplePlugin_iBusRsp_stages_0_output_ready && _zz_100_);
    if(IBusSimplePlugin_mmuBus_busy)begin
      IBusSimplePlugin_iBusRsp_stages_0_input_ready = 1'b0;
    end
  end

  assign IBusSimplePlugin_iBusRsp_stages_0_output_valid = (IBusSimplePlugin_iBusRsp_stages_0_input_valid && _zz_100_);
  assign IBusSimplePlugin_iBusRsp_stages_0_output_payload = IBusSimplePlugin_iBusRsp_stages_0_input_payload;
  assign IBusSimplePlugin_iBusRsp_stages_1_halt = 1'b0;
  assign _zz_101_ = (! IBusSimplePlugin_iBusRsp_stages_1_halt);
  assign IBusSimplePlugin_iBusRsp_stages_1_input_ready = (IBusSimplePlugin_iBusRsp_stages_1_output_ready && _zz_101_);
  assign IBusSimplePlugin_iBusRsp_stages_1_output_valid = (IBusSimplePlugin_iBusRsp_stages_1_input_valid && _zz_101_);
  assign IBusSimplePlugin_iBusRsp_stages_1_output_payload = IBusSimplePlugin_iBusRsp_stages_1_input_payload;
  assign IBusSimplePlugin_iBusRsp_stages_0_output_ready = _zz_102_;
  assign _zz_102_ = ((1'b0 && (! _zz_103_)) || IBusSimplePlugin_iBusRsp_stages_1_input_ready);
  assign _zz_103_ = _zz_104_;
  assign IBusSimplePlugin_iBusRsp_stages_1_input_valid = _zz_103_;
  assign IBusSimplePlugin_iBusRsp_stages_1_input_payload = IBusSimplePlugin_fetchPc_pcReg;
  always @ (*) begin
    IBusSimplePlugin_iBusRsp_readyForError = 1'b1;
    if(IBusSimplePlugin_injector_decodeInput_valid)begin
      IBusSimplePlugin_iBusRsp_readyForError = 1'b0;
    end
    if((! IBusSimplePlugin_pcValids_0))begin
      IBusSimplePlugin_iBusRsp_readyForError = 1'b0;
    end
  end

  assign IBusSimplePlugin_iBusRsp_inputBeforeStage_ready = ((1'b0 && (! IBusSimplePlugin_injector_decodeInput_valid)) || IBusSimplePlugin_injector_decodeInput_ready);
  assign IBusSimplePlugin_injector_decodeInput_valid = _zz_105_;
  assign IBusSimplePlugin_injector_decodeInput_payload_pc = _zz_106_;
  assign IBusSimplePlugin_injector_decodeInput_payload_rsp_error = _zz_107_;
  assign IBusSimplePlugin_injector_decodeInput_payload_rsp_inst = _zz_108_;
  assign IBusSimplePlugin_injector_decodeInput_payload_isRvc = _zz_109_;
  assign _zz_94_ = (decode_arbitration_isStuck ? decode_INSTRUCTION : IBusSimplePlugin_iBusRsp_inputBeforeStage_payload_rsp_inst);
  assign IBusSimplePlugin_pcValids_0 = IBusSimplePlugin_injector_nextPcCalc_valids_1;
  assign IBusSimplePlugin_pcValids_1 = IBusSimplePlugin_injector_nextPcCalc_valids_2;
  assign IBusSimplePlugin_pcValids_2 = IBusSimplePlugin_injector_nextPcCalc_valids_3;
  assign IBusSimplePlugin_pcValids_3 = IBusSimplePlugin_injector_nextPcCalc_valids_4;
  assign IBusSimplePlugin_injector_decodeInput_ready = (! decode_arbitration_isStuck);
  assign decode_arbitration_isValid = (IBusSimplePlugin_injector_decodeInput_valid && (! IBusSimplePlugin_injector_decodeRemoved));
  assign _zz_93_ = IBusSimplePlugin_injector_decodeInput_payload_pc;
  assign _zz_92_ = IBusSimplePlugin_injector_decodeInput_payload_rsp_inst;
  assign _zz_91_ = (decode_PC + (32'b00000000000000000000000000000100));
  assign iBus_cmd_valid = IBusSimplePlugin_cmd_valid;
  assign IBusSimplePlugin_cmd_ready = iBus_cmd_ready;
  assign iBus_cmd_payload_pc = IBusSimplePlugin_cmd_payload_pc;
  assign IBusSimplePlugin_pendingCmdNext = (_zz_197_ - _zz_201_);
  always @ (*) begin
    IBusSimplePlugin_cmd_valid = ((IBusSimplePlugin_iBusRsp_stages_0_input_valid && IBusSimplePlugin_iBusRsp_stages_0_output_ready) && (IBusSimplePlugin_pendingCmd != (3'b111)));
    if(_zz_175_)begin
      IBusSimplePlugin_cmd_valid = 1'b0;
    end
  end

  assign IBusSimplePlugin_mmuBus_cmd_isValid = IBusSimplePlugin_iBusRsp_stages_0_input_valid;
  assign IBusSimplePlugin_mmuBus_cmd_virtualAddress = IBusSimplePlugin_iBusRsp_stages_0_input_payload;
  assign IBusSimplePlugin_mmuBus_cmd_bypassTranslation = 1'b0;
  assign IBusSimplePlugin_mmuBus_end = ((IBusSimplePlugin_iBusRsp_stages_0_output_valid && IBusSimplePlugin_iBusRsp_stages_0_output_ready) || IBusSimplePlugin_fetcherflushIt);
  assign IBusSimplePlugin_cmd_payload_pc = {IBusSimplePlugin_mmuBus_rsp_physicalAddress[31 : 2],(2'b00)};
  assign iBus_rsp_takeWhen_valid = (iBus_rsp_valid && (! (IBusSimplePlugin_rspJoin_discardCounter != (3'b000))));
  assign iBus_rsp_takeWhen_payload_error = iBus_rsp_payload_error;
  assign iBus_rsp_takeWhen_payload_inst = iBus_rsp_payload_inst;
  assign IBusSimplePlugin_rspJoin_rspBufferOutput_valid = IBusSimplePlugin_rspJoin_rspBuffer_c_io_pop_valid;
  assign IBusSimplePlugin_rspJoin_rspBufferOutput_payload_error = IBusSimplePlugin_rspJoin_rspBuffer_c_io_pop_payload_error;
  assign IBusSimplePlugin_rspJoin_rspBufferOutput_payload_inst = IBusSimplePlugin_rspJoin_rspBuffer_c_io_pop_payload_inst;
  assign IBusSimplePlugin_rspJoin_fetchRsp_pc = IBusSimplePlugin_iBusRsp_stages_1_output_payload;
  always @ (*) begin
    IBusSimplePlugin_rspJoin_fetchRsp_rsp_error = IBusSimplePlugin_rspJoin_rspBufferOutput_payload_error;
    if((! IBusSimplePlugin_rspJoin_rspBufferOutput_valid))begin
      IBusSimplePlugin_rspJoin_fetchRsp_rsp_error = 1'b0;
    end
  end

  assign IBusSimplePlugin_rspJoin_fetchRsp_rsp_inst = IBusSimplePlugin_rspJoin_rspBufferOutput_payload_inst;
  always @ (*) begin
    IBusSimplePlugin_rspJoin_exceptionDetected = 1'b0;
    if(_zz_176_)begin
      IBusSimplePlugin_rspJoin_exceptionDetected = 1'b1;
    end
  end

  always @ (*) begin
    IBusSimplePlugin_rspJoin_redoRequired = 1'b0;
    if((IBusSimplePlugin_iBusRsp_stages_1_input_valid && IBusSimplePlugin_mmu_joinCtx_refilling))begin
      IBusSimplePlugin_rspJoin_redoRequired = 1'b1;
    end
  end

  assign IBusSimplePlugin_rspJoin_join_valid = (IBusSimplePlugin_iBusRsp_stages_1_output_valid && IBusSimplePlugin_rspJoin_rspBufferOutput_valid);
  assign IBusSimplePlugin_rspJoin_join_payload_pc = IBusSimplePlugin_rspJoin_fetchRsp_pc;
  assign IBusSimplePlugin_rspJoin_join_payload_rsp_error = IBusSimplePlugin_rspJoin_fetchRsp_rsp_error;
  assign IBusSimplePlugin_rspJoin_join_payload_rsp_inst = IBusSimplePlugin_rspJoin_fetchRsp_rsp_inst;
  assign IBusSimplePlugin_rspJoin_join_payload_isRvc = IBusSimplePlugin_rspJoin_fetchRsp_isRvc;
  assign IBusSimplePlugin_iBusRsp_stages_1_output_ready = (IBusSimplePlugin_iBusRsp_stages_1_output_valid ? (IBusSimplePlugin_rspJoin_join_valid && IBusSimplePlugin_rspJoin_join_ready) : IBusSimplePlugin_rspJoin_join_ready);
  assign IBusSimplePlugin_rspJoin_rspBufferOutput_ready = (IBusSimplePlugin_rspJoin_join_valid && IBusSimplePlugin_rspJoin_join_ready);
  assign _zz_110_ = (! (IBusSimplePlugin_rspJoin_exceptionDetected || IBusSimplePlugin_rspJoin_redoRequired));
  assign IBusSimplePlugin_rspJoin_join_ready = (IBusSimplePlugin_iBusRsp_inputBeforeStage_ready && _zz_110_);
  assign IBusSimplePlugin_iBusRsp_inputBeforeStage_valid = (IBusSimplePlugin_rspJoin_join_valid && _zz_110_);
  assign IBusSimplePlugin_iBusRsp_inputBeforeStage_payload_pc = IBusSimplePlugin_rspJoin_join_payload_pc;
  assign IBusSimplePlugin_iBusRsp_inputBeforeStage_payload_rsp_error = IBusSimplePlugin_rspJoin_join_payload_rsp_error;
  assign IBusSimplePlugin_iBusRsp_inputBeforeStage_payload_rsp_inst = IBusSimplePlugin_rspJoin_join_payload_rsp_inst;
  assign IBusSimplePlugin_iBusRsp_inputBeforeStage_payload_isRvc = IBusSimplePlugin_rspJoin_join_payload_isRvc;
  assign IBusSimplePlugin_redoBranch_valid = (IBusSimplePlugin_rspJoin_redoRequired && IBusSimplePlugin_iBusRsp_readyForError);
  assign IBusSimplePlugin_redoBranch_payload = decode_PC;
  always @ (*) begin
    IBusSimplePlugin_decodeExceptionPort_payload_code = (4'bxxxx);
    if(_zz_176_)begin
      IBusSimplePlugin_decodeExceptionPort_payload_code = (4'b1100);
    end
  end

  assign IBusSimplePlugin_decodeExceptionPort_payload_badAddr = {IBusSimplePlugin_rspJoin_join_payload_pc[31 : 2],(2'b00)};
  assign IBusSimplePlugin_decodeExceptionPort_valid = (IBusSimplePlugin_rspJoin_exceptionDetected && IBusSimplePlugin_iBusRsp_readyForError);
  assign _zz_111_ = 1'b0;
  assign _zz_88_ = (((dBus_cmd_payload_size == (2'b10)) && (dBus_cmd_payload_address[1 : 0] != (2'b00))) || ((dBus_cmd_payload_size == (2'b01)) && (dBus_cmd_payload_address[0 : 0] != (1'b0))));
  always @ (*) begin
    execute_DBusSimplePlugin_skipCmd = 1'b0;
    if(execute_ALIGNEMENT_FAULT)begin
      execute_DBusSimplePlugin_skipCmd = 1'b1;
    end
    if((execute_MMU_FAULT || execute_MMU_RSP_refilling))begin
      execute_DBusSimplePlugin_skipCmd = 1'b1;
    end
  end

  assign dBus_cmd_valid = (((((execute_arbitration_isValid && execute_MEMORY_ENABLE) && (! execute_arbitration_isStuckByOthers)) && (! execute_arbitration_isFlushed)) && (! execute_DBusSimplePlugin_skipCmd)) && (! _zz_111_));
  assign dBus_cmd_payload_wr = execute_MEMORY_STORE;
  assign dBus_cmd_payload_size = execute_INSTRUCTION[13 : 12];
  always @ (*) begin
    case(dBus_cmd_payload_size)
      2'b00 : begin
        _zz_112_ = {{{execute_RS2[7 : 0],execute_RS2[7 : 0]},execute_RS2[7 : 0]},execute_RS2[7 : 0]};
      end
      2'b01 : begin
        _zz_112_ = {execute_RS2[15 : 0],execute_RS2[15 : 0]};
      end
      default : begin
        _zz_112_ = execute_RS2[31 : 0];
      end
    endcase
  end

  assign dBus_cmd_payload_data = _zz_112_;
  assign _zz_87_ = dBus_cmd_payload_address[1 : 0];
  always @ (*) begin
    case(dBus_cmd_payload_size)
      2'b00 : begin
        _zz_113_ = (4'b0001);
      end
      2'b01 : begin
        _zz_113_ = (4'b0011);
      end
      default : begin
        _zz_113_ = (4'b1111);
      end
    endcase
  end

  assign execute_DBusSimplePlugin_formalMask = (_zz_113_ <<< dBus_cmd_payload_address[1 : 0]);
  assign DBusSimplePlugin_mmuBus_cmd_isValid = (execute_arbitration_isValid && execute_MEMORY_ENABLE);
  assign DBusSimplePlugin_mmuBus_cmd_virtualAddress = execute_SRC_ADD;
  assign DBusSimplePlugin_mmuBus_cmd_bypassTranslation = 1'b0;
  assign DBusSimplePlugin_mmuBus_end = ((! execute_arbitration_isStuck) || execute_arbitration_removeIt);
  assign dBus_cmd_payload_address = DBusSimplePlugin_mmuBus_rsp_physicalAddress;
  assign _zz_86_ = ((execute_MMU_RSP_exception || ((! execute_MMU_RSP_allowWrite) && execute_MEMORY_STORE)) || ((! execute_MMU_RSP_allowRead) && (! execute_MEMORY_STORE)));
  assign _zz_79_ = DBusSimplePlugin_mmuBus_rsp_physicalAddress;
  assign _zz_80_ = DBusSimplePlugin_mmuBus_rsp_isIoAccess;
  assign _zz_81_ = DBusSimplePlugin_mmuBus_rsp_allowRead;
  assign _zz_82_ = DBusSimplePlugin_mmuBus_rsp_allowWrite;
  assign _zz_83_ = DBusSimplePlugin_mmuBus_rsp_allowExecute;
  assign _zz_84_ = DBusSimplePlugin_mmuBus_rsp_exception;
  assign _zz_85_ = DBusSimplePlugin_mmuBus_rsp_refilling;
  assign _zz_78_ = dBus_rsp_data;
  always @ (*) begin
    DBusSimplePlugin_memoryExceptionPort_valid = 1'b0;
    if(_zz_177_)begin
      DBusSimplePlugin_memoryExceptionPort_valid = 1'b1;
    end
    if(memory_ALIGNEMENT_FAULT)begin
      DBusSimplePlugin_memoryExceptionPort_valid = 1'b1;
    end
    if(memory_MMU_RSP_refilling)begin
      DBusSimplePlugin_memoryExceptionPort_valid = 1'b0;
    end else begin
      if(memory_MMU_FAULT)begin
        DBusSimplePlugin_memoryExceptionPort_valid = 1'b1;
      end
    end
    if(_zz_178_)begin
      DBusSimplePlugin_memoryExceptionPort_valid = 1'b0;
    end
  end

  always @ (*) begin
    DBusSimplePlugin_memoryExceptionPort_payload_code = (4'bxxxx);
    if(_zz_177_)begin
      DBusSimplePlugin_memoryExceptionPort_payload_code = (4'b0101);
    end
    if(memory_ALIGNEMENT_FAULT)begin
      DBusSimplePlugin_memoryExceptionPort_payload_code = {1'd0, _zz_206_};
    end
    if(! memory_MMU_RSP_refilling) begin
      if(memory_MMU_FAULT)begin
        DBusSimplePlugin_memoryExceptionPort_payload_code = (memory_MEMORY_STORE ? (4'b1111) : (4'b1101));
      end
    end
  end

  assign DBusSimplePlugin_memoryExceptionPort_payload_badAddr = memory_REGFILE_WRITE_DATA;
  always @ (*) begin
    DBusSimplePlugin_redoBranch_valid = 1'b0;
    if(memory_MMU_RSP_refilling)begin
      DBusSimplePlugin_redoBranch_valid = 1'b1;
    end
    if(_zz_178_)begin
      DBusSimplePlugin_redoBranch_valid = 1'b0;
    end
  end

  assign DBusSimplePlugin_redoBranch_payload = memory_PC;
  always @ (*) begin
    writeBack_DBusSimplePlugin_rspShifted = writeBack_MEMORY_READ_DATA;
    case(writeBack_MEMORY_ADDRESS_LOW)
      2'b01 : begin
        writeBack_DBusSimplePlugin_rspShifted[7 : 0] = writeBack_MEMORY_READ_DATA[15 : 8];
      end
      2'b10 : begin
        writeBack_DBusSimplePlugin_rspShifted[15 : 0] = writeBack_MEMORY_READ_DATA[31 : 16];
      end
      2'b11 : begin
        writeBack_DBusSimplePlugin_rspShifted[7 : 0] = writeBack_MEMORY_READ_DATA[31 : 24];
      end
      default : begin
      end
    endcase
  end

  assign _zz_114_ = (writeBack_DBusSimplePlugin_rspShifted[7] && (! writeBack_INSTRUCTION[14]));
  always @ (*) begin
    _zz_115_[31] = _zz_114_;
    _zz_115_[30] = _zz_114_;
    _zz_115_[29] = _zz_114_;
    _zz_115_[28] = _zz_114_;
    _zz_115_[27] = _zz_114_;
    _zz_115_[26] = _zz_114_;
    _zz_115_[25] = _zz_114_;
    _zz_115_[24] = _zz_114_;
    _zz_115_[23] = _zz_114_;
    _zz_115_[22] = _zz_114_;
    _zz_115_[21] = _zz_114_;
    _zz_115_[20] = _zz_114_;
    _zz_115_[19] = _zz_114_;
    _zz_115_[18] = _zz_114_;
    _zz_115_[17] = _zz_114_;
    _zz_115_[16] = _zz_114_;
    _zz_115_[15] = _zz_114_;
    _zz_115_[14] = _zz_114_;
    _zz_115_[13] = _zz_114_;
    _zz_115_[12] = _zz_114_;
    _zz_115_[11] = _zz_114_;
    _zz_115_[10] = _zz_114_;
    _zz_115_[9] = _zz_114_;
    _zz_115_[8] = _zz_114_;
    _zz_115_[7 : 0] = writeBack_DBusSimplePlugin_rspShifted[7 : 0];
  end

  assign _zz_116_ = (writeBack_DBusSimplePlugin_rspShifted[15] && (! writeBack_INSTRUCTION[14]));
  always @ (*) begin
    _zz_117_[31] = _zz_116_;
    _zz_117_[30] = _zz_116_;
    _zz_117_[29] = _zz_116_;
    _zz_117_[28] = _zz_116_;
    _zz_117_[27] = _zz_116_;
    _zz_117_[26] = _zz_116_;
    _zz_117_[25] = _zz_116_;
    _zz_117_[24] = _zz_116_;
    _zz_117_[23] = _zz_116_;
    _zz_117_[22] = _zz_116_;
    _zz_117_[21] = _zz_116_;
    _zz_117_[20] = _zz_116_;
    _zz_117_[19] = _zz_116_;
    _zz_117_[18] = _zz_116_;
    _zz_117_[17] = _zz_116_;
    _zz_117_[16] = _zz_116_;
    _zz_117_[15 : 0] = writeBack_DBusSimplePlugin_rspShifted[15 : 0];
  end

  always @ (*) begin
    case(_zz_192_)
      2'b00 : begin
        writeBack_DBusSimplePlugin_rspFormated = _zz_115_;
      end
      2'b01 : begin
        writeBack_DBusSimplePlugin_rspFormated = _zz_117_;
      end
      default : begin
        writeBack_DBusSimplePlugin_rspFormated = writeBack_DBusSimplePlugin_rspShifted;
      end
    endcase
  end

  assign IBusSimplePlugin_mmuBus_rsp_physicalAddress = IBusSimplePlugin_mmuBus_cmd_virtualAddress;
  assign IBusSimplePlugin_mmuBus_rsp_allowRead = 1'b1;
  assign IBusSimplePlugin_mmuBus_rsp_allowWrite = 1'b1;
  assign IBusSimplePlugin_mmuBus_rsp_allowExecute = 1'b1;
  assign IBusSimplePlugin_mmuBus_rsp_isIoAccess = IBusSimplePlugin_mmuBus_rsp_physicalAddress[31];
  assign IBusSimplePlugin_mmuBus_rsp_exception = 1'b0;
  assign IBusSimplePlugin_mmuBus_rsp_refilling = 1'b0;
  assign IBusSimplePlugin_mmuBus_busy = 1'b0;
  assign DBusSimplePlugin_mmuBus_rsp_physicalAddress = DBusSimplePlugin_mmuBus_cmd_virtualAddress;
  assign DBusSimplePlugin_mmuBus_rsp_allowRead = 1'b1;
  assign DBusSimplePlugin_mmuBus_rsp_allowWrite = 1'b1;
  assign DBusSimplePlugin_mmuBus_rsp_allowExecute = 1'b1;
  assign DBusSimplePlugin_mmuBus_rsp_isIoAccess = DBusSimplePlugin_mmuBus_rsp_physicalAddress[31];
  assign DBusSimplePlugin_mmuBus_rsp_exception = 1'b0;
  assign DBusSimplePlugin_mmuBus_rsp_refilling = 1'b0;
  assign DBusSimplePlugin_mmuBus_busy = 1'b0;
  assign _zz_119_ = ((decode_INSTRUCTION & (32'b00000000000000000000000001001000)) == (32'b00000000000000000000000001001000));
  assign _zz_120_ = ((decode_INSTRUCTION & (32'b00000000000000000000000000000100)) == (32'b00000000000000000000000000000100));
  assign _zz_121_ = ((decode_INSTRUCTION & (32'b00000000000000000100000001010000)) == (32'b00000000000000000100000001010000));
  assign _zz_118_ = {({_zz_120_,{_zz_250_,_zz_251_}} != (3'b000)),{((_zz_252_ == _zz_253_) != (1'b0)),{(_zz_254_ != (1'b0)),{(_zz_255_ != _zz_256_),{_zz_257_,{_zz_258_,_zz_259_}}}}}};
  assign _zz_76_ = ({((decode_INSTRUCTION & (32'b00000000000000000000000001011111)) == (32'b00000000000000000000000000010111)),{((decode_INSTRUCTION & (32'b00000000000000000000000001111111)) == (32'b00000000000000000000000001101111)),{((decode_INSTRUCTION & (32'b00000000000000000001000001101111)) == (32'b00000000000000000000000000000011)),{((decode_INSTRUCTION & _zz_358_) == (32'b00000000000000000001000001110011)),{(_zz_359_ == _zz_360_),{_zz_361_,{_zz_362_,_zz_363_}}}}}}} != (20'b00000000000000000000));
  assign _zz_122_ = _zz_118_[1 : 0];
  assign _zz_75_ = _zz_122_;
  assign _zz_74_ = _zz_207_[0];
  assign _zz_73_ = _zz_208_[0];
  assign _zz_123_ = _zz_118_[5 : 4];
  assign _zz_72_ = _zz_123_;
  assign _zz_124_ = _zz_118_[7 : 6];
  assign _zz_71_ = _zz_124_;
  assign _zz_70_ = _zz_209_[0];
  assign _zz_69_ = _zz_210_[0];
  assign _zz_125_ = _zz_118_[11 : 10];
  assign _zz_68_ = _zz_125_;
  assign _zz_67_ = _zz_211_[0];
  assign _zz_66_ = _zz_212_[0];
  assign _zz_126_ = _zz_118_[15 : 14];
  assign _zz_65_ = _zz_126_;
  assign _zz_127_ = _zz_118_[17 : 16];
  assign _zz_64_ = _zz_127_;
  assign _zz_63_ = _zz_213_[0];
  assign _zz_128_ = _zz_118_[20 : 19];
  assign _zz_62_ = _zz_128_;
  assign _zz_61_ = _zz_214_[0];
  assign _zz_60_ = _zz_215_[0];
  assign _zz_59_ = _zz_216_[0];
  assign _zz_58_ = _zz_217_[0];
  assign decodeExceptionPort_valid = ((decode_arbitration_isValid && decode_INSTRUCTION_READY) && (! decode_LEGAL_INSTRUCTION));
  assign decodeExceptionPort_payload_code = (4'b0010);
  assign decodeExceptionPort_payload_badAddr = decode_INSTRUCTION;
  assign decode_RegFilePlugin_regFileReadAddress1 = decode_INSTRUCTION_ANTICIPATED[19 : 15];
  assign decode_RegFilePlugin_regFileReadAddress2 = decode_INSTRUCTION_ANTICIPATED[24 : 20];
  assign decode_RegFilePlugin_rs1Data = _zz_166_;
  assign decode_RegFilePlugin_rs2Data = _zz_167_;
  assign _zz_57_ = decode_RegFilePlugin_rs1Data;
  assign _zz_56_ = decode_RegFilePlugin_rs2Data;
  always @ (*) begin
    lastStageRegFileWrite_valid = (_zz_54_ && writeBack_arbitration_isFiring);
    if(_zz_129_)begin
      lastStageRegFileWrite_valid = 1'b1;
    end
  end

  assign lastStageRegFileWrite_payload_address = _zz_53_[11 : 7];
  assign lastStageRegFileWrite_payload_data = _zz_77_;
  always @ (*) begin
    case(execute_ALU_BITWISE_CTRL)
      `AluBitwiseCtrlEnum_defaultEncoding_AND_1 : begin
        execute_IntAluPlugin_bitwise = (execute_SRC1 & execute_SRC2);
      end
      `AluBitwiseCtrlEnum_defaultEncoding_OR_1 : begin
        execute_IntAluPlugin_bitwise = (execute_SRC1 | execute_SRC2);
      end
      default : begin
        execute_IntAluPlugin_bitwise = (execute_SRC1 ^ execute_SRC2);
      end
    endcase
  end

  always @ (*) begin
    case(execute_ALU_CTRL)
      `AluCtrlEnum_defaultEncoding_BITWISE : begin
        _zz_130_ = execute_IntAluPlugin_bitwise;
      end
      `AluCtrlEnum_defaultEncoding_SLT_SLTU : begin
        _zz_130_ = {31'd0, _zz_218_};
      end
      default : begin
        _zz_130_ = execute_SRC_ADD_SUB;
      end
    endcase
  end

  assign _zz_51_ = _zz_130_;
  assign _zz_49_ = (decode_SRC_ADD_ZERO && (! decode_SRC_USE_SUB_LESS));
  always @ (*) begin
    case(execute_SRC1_CTRL)
      `Src1CtrlEnum_defaultEncoding_RS : begin
        _zz_131_ = execute_RS1;
      end
      `Src1CtrlEnum_defaultEncoding_PC_INCREMENT : begin
        _zz_131_ = {29'd0, _zz_219_};
      end
      `Src1CtrlEnum_defaultEncoding_IMU : begin
        _zz_131_ = {execute_INSTRUCTION[31 : 12],(12'b000000000000)};
      end
      default : begin
        _zz_131_ = {27'd0, _zz_220_};
      end
    endcase
  end

  assign _zz_48_ = _zz_131_;
  assign _zz_132_ = _zz_221_[11];
  always @ (*) begin
    _zz_133_[19] = _zz_132_;
    _zz_133_[18] = _zz_132_;
    _zz_133_[17] = _zz_132_;
    _zz_133_[16] = _zz_132_;
    _zz_133_[15] = _zz_132_;
    _zz_133_[14] = _zz_132_;
    _zz_133_[13] = _zz_132_;
    _zz_133_[12] = _zz_132_;
    _zz_133_[11] = _zz_132_;
    _zz_133_[10] = _zz_132_;
    _zz_133_[9] = _zz_132_;
    _zz_133_[8] = _zz_132_;
    _zz_133_[7] = _zz_132_;
    _zz_133_[6] = _zz_132_;
    _zz_133_[5] = _zz_132_;
    _zz_133_[4] = _zz_132_;
    _zz_133_[3] = _zz_132_;
    _zz_133_[2] = _zz_132_;
    _zz_133_[1] = _zz_132_;
    _zz_133_[0] = _zz_132_;
  end

  assign _zz_134_ = _zz_222_[11];
  always @ (*) begin
    _zz_135_[19] = _zz_134_;
    _zz_135_[18] = _zz_134_;
    _zz_135_[17] = _zz_134_;
    _zz_135_[16] = _zz_134_;
    _zz_135_[15] = _zz_134_;
    _zz_135_[14] = _zz_134_;
    _zz_135_[13] = _zz_134_;
    _zz_135_[12] = _zz_134_;
    _zz_135_[11] = _zz_134_;
    _zz_135_[10] = _zz_134_;
    _zz_135_[9] = _zz_134_;
    _zz_135_[8] = _zz_134_;
    _zz_135_[7] = _zz_134_;
    _zz_135_[6] = _zz_134_;
    _zz_135_[5] = _zz_134_;
    _zz_135_[4] = _zz_134_;
    _zz_135_[3] = _zz_134_;
    _zz_135_[2] = _zz_134_;
    _zz_135_[1] = _zz_134_;
    _zz_135_[0] = _zz_134_;
  end

  always @ (*) begin
    case(execute_SRC2_CTRL)
      `Src2CtrlEnum_defaultEncoding_RS : begin
        _zz_136_ = execute_RS2;
      end
      `Src2CtrlEnum_defaultEncoding_IMI : begin
        _zz_136_ = {_zz_133_,execute_INSTRUCTION[31 : 20]};
      end
      `Src2CtrlEnum_defaultEncoding_IMS : begin
        _zz_136_ = {_zz_135_,{execute_INSTRUCTION[31 : 25],execute_INSTRUCTION[11 : 7]}};
      end
      default : begin
        _zz_136_ = _zz_44_;
      end
    endcase
  end

  assign _zz_46_ = _zz_136_;
  always @ (*) begin
    execute_SrcPlugin_addSub = _zz_223_;
    if(execute_SRC2_FORCE_ZERO)begin
      execute_SrcPlugin_addSub = execute_SRC1;
    end
  end

  assign execute_SrcPlugin_less = ((execute_SRC1[31] == execute_SRC2[31]) ? execute_SrcPlugin_addSub[31] : (execute_SRC_LESS_UNSIGNED ? execute_SRC2[31] : execute_SRC1[31]));
  assign _zz_43_ = execute_SrcPlugin_addSub;
  assign _zz_42_ = execute_SrcPlugin_addSub;
  assign _zz_41_ = execute_SrcPlugin_less;
  assign execute_FullBarrelShifterPlugin_amplitude = execute_SRC2[4 : 0];
  always @ (*) begin
    _zz_137_[0] = execute_SRC1[31];
    _zz_137_[1] = execute_SRC1[30];
    _zz_137_[2] = execute_SRC1[29];
    _zz_137_[3] = execute_SRC1[28];
    _zz_137_[4] = execute_SRC1[27];
    _zz_137_[5] = execute_SRC1[26];
    _zz_137_[6] = execute_SRC1[25];
    _zz_137_[7] = execute_SRC1[24];
    _zz_137_[8] = execute_SRC1[23];
    _zz_137_[9] = execute_SRC1[22];
    _zz_137_[10] = execute_SRC1[21];
    _zz_137_[11] = execute_SRC1[20];
    _zz_137_[12] = execute_SRC1[19];
    _zz_137_[13] = execute_SRC1[18];
    _zz_137_[14] = execute_SRC1[17];
    _zz_137_[15] = execute_SRC1[16];
    _zz_137_[16] = execute_SRC1[15];
    _zz_137_[17] = execute_SRC1[14];
    _zz_137_[18] = execute_SRC1[13];
    _zz_137_[19] = execute_SRC1[12];
    _zz_137_[20] = execute_SRC1[11];
    _zz_137_[21] = execute_SRC1[10];
    _zz_137_[22] = execute_SRC1[9];
    _zz_137_[23] = execute_SRC1[8];
    _zz_137_[24] = execute_SRC1[7];
    _zz_137_[25] = execute_SRC1[6];
    _zz_137_[26] = execute_SRC1[5];
    _zz_137_[27] = execute_SRC1[4];
    _zz_137_[28] = execute_SRC1[3];
    _zz_137_[29] = execute_SRC1[2];
    _zz_137_[30] = execute_SRC1[1];
    _zz_137_[31] = execute_SRC1[0];
  end

  assign execute_FullBarrelShifterPlugin_reversed = ((execute_SHIFT_CTRL == `ShiftCtrlEnum_defaultEncoding_SLL_1) ? _zz_137_ : execute_SRC1);
  assign _zz_39_ = _zz_231_;
  always @ (*) begin
    _zz_138_[0] = memory_SHIFT_RIGHT[31];
    _zz_138_[1] = memory_SHIFT_RIGHT[30];
    _zz_138_[2] = memory_SHIFT_RIGHT[29];
    _zz_138_[3] = memory_SHIFT_RIGHT[28];
    _zz_138_[4] = memory_SHIFT_RIGHT[27];
    _zz_138_[5] = memory_SHIFT_RIGHT[26];
    _zz_138_[6] = memory_SHIFT_RIGHT[25];
    _zz_138_[7] = memory_SHIFT_RIGHT[24];
    _zz_138_[8] = memory_SHIFT_RIGHT[23];
    _zz_138_[9] = memory_SHIFT_RIGHT[22];
    _zz_138_[10] = memory_SHIFT_RIGHT[21];
    _zz_138_[11] = memory_SHIFT_RIGHT[20];
    _zz_138_[12] = memory_SHIFT_RIGHT[19];
    _zz_138_[13] = memory_SHIFT_RIGHT[18];
    _zz_138_[14] = memory_SHIFT_RIGHT[17];
    _zz_138_[15] = memory_SHIFT_RIGHT[16];
    _zz_138_[16] = memory_SHIFT_RIGHT[15];
    _zz_138_[17] = memory_SHIFT_RIGHT[14];
    _zz_138_[18] = memory_SHIFT_RIGHT[13];
    _zz_138_[19] = memory_SHIFT_RIGHT[12];
    _zz_138_[20] = memory_SHIFT_RIGHT[11];
    _zz_138_[21] = memory_SHIFT_RIGHT[10];
    _zz_138_[22] = memory_SHIFT_RIGHT[9];
    _zz_138_[23] = memory_SHIFT_RIGHT[8];
    _zz_138_[24] = memory_SHIFT_RIGHT[7];
    _zz_138_[25] = memory_SHIFT_RIGHT[6];
    _zz_138_[26] = memory_SHIFT_RIGHT[5];
    _zz_138_[27] = memory_SHIFT_RIGHT[4];
    _zz_138_[28] = memory_SHIFT_RIGHT[3];
    _zz_138_[29] = memory_SHIFT_RIGHT[2];
    _zz_138_[30] = memory_SHIFT_RIGHT[1];
    _zz_138_[31] = memory_SHIFT_RIGHT[0];
  end

  always @ (*) begin
    _zz_139_ = 1'b0;
    if(_zz_142_)begin
      if((_zz_143_ == decode_INSTRUCTION[19 : 15]))begin
        _zz_139_ = 1'b1;
      end
    end
    if(_zz_179_)begin
      if(_zz_180_)begin
        if((writeBack_INSTRUCTION[11 : 7] == decode_INSTRUCTION[19 : 15]))begin
          _zz_139_ = 1'b1;
        end
      end
    end
    if(_zz_181_)begin
      if(_zz_182_)begin
        if((memory_INSTRUCTION[11 : 7] == decode_INSTRUCTION[19 : 15]))begin
          _zz_139_ = 1'b1;
        end
      end
    end
    if(_zz_183_)begin
      if(_zz_184_)begin
        if((execute_INSTRUCTION[11 : 7] == decode_INSTRUCTION[19 : 15]))begin
          _zz_139_ = 1'b1;
        end
      end
    end
    if((! decode_RS1_USE))begin
      _zz_139_ = 1'b0;
    end
  end

  always @ (*) begin
    _zz_140_ = 1'b0;
    if(_zz_142_)begin
      if((_zz_143_ == decode_INSTRUCTION[24 : 20]))begin
        _zz_140_ = 1'b1;
      end
    end
    if(_zz_179_)begin
      if(_zz_180_)begin
        if((writeBack_INSTRUCTION[11 : 7] == decode_INSTRUCTION[24 : 20]))begin
          _zz_140_ = 1'b1;
        end
      end
    end
    if(_zz_181_)begin
      if(_zz_182_)begin
        if((memory_INSTRUCTION[11 : 7] == decode_INSTRUCTION[24 : 20]))begin
          _zz_140_ = 1'b1;
        end
      end
    end
    if(_zz_183_)begin
      if(_zz_184_)begin
        if((execute_INSTRUCTION[11 : 7] == decode_INSTRUCTION[24 : 20]))begin
          _zz_140_ = 1'b1;
        end
      end
    end
    if((! decode_RS2_USE))begin
      _zz_140_ = 1'b0;
    end
  end

  assign _zz_141_ = (_zz_54_ && writeBack_arbitration_isFiring);
  assign execute_BranchPlugin_eq = (execute_SRC1 == execute_SRC2);
  assign _zz_144_ = execute_INSTRUCTION[14 : 12];
  always @ (*) begin
    if((_zz_144_ == (3'b000))) begin
        _zz_145_ = execute_BranchPlugin_eq;
    end else if((_zz_144_ == (3'b001))) begin
        _zz_145_ = (! execute_BranchPlugin_eq);
    end else if((((_zz_144_ & (3'b101)) == (3'b101)))) begin
        _zz_145_ = (! execute_SRC_LESS);
    end else begin
        _zz_145_ = execute_SRC_LESS;
    end
  end

  always @ (*) begin
    case(execute_BRANCH_CTRL)
      `BranchCtrlEnum_defaultEncoding_INC : begin
        _zz_146_ = 1'b0;
      end
      `BranchCtrlEnum_defaultEncoding_JAL : begin
        _zz_146_ = 1'b1;
      end
      `BranchCtrlEnum_defaultEncoding_JALR : begin
        _zz_146_ = 1'b1;
      end
      default : begin
        _zz_146_ = _zz_145_;
      end
    endcase
  end

  assign _zz_36_ = _zz_146_;
  assign execute_BranchPlugin_branch_src1 = ((execute_BRANCH_CTRL == `BranchCtrlEnum_defaultEncoding_JALR) ? execute_RS1 : execute_PC);
  assign _zz_147_ = _zz_233_[19];
  always @ (*) begin
    _zz_148_[10] = _zz_147_;
    _zz_148_[9] = _zz_147_;
    _zz_148_[8] = _zz_147_;
    _zz_148_[7] = _zz_147_;
    _zz_148_[6] = _zz_147_;
    _zz_148_[5] = _zz_147_;
    _zz_148_[4] = _zz_147_;
    _zz_148_[3] = _zz_147_;
    _zz_148_[2] = _zz_147_;
    _zz_148_[1] = _zz_147_;
    _zz_148_[0] = _zz_147_;
  end

  assign _zz_149_ = _zz_234_[11];
  always @ (*) begin
    _zz_150_[19] = _zz_149_;
    _zz_150_[18] = _zz_149_;
    _zz_150_[17] = _zz_149_;
    _zz_150_[16] = _zz_149_;
    _zz_150_[15] = _zz_149_;
    _zz_150_[14] = _zz_149_;
    _zz_150_[13] = _zz_149_;
    _zz_150_[12] = _zz_149_;
    _zz_150_[11] = _zz_149_;
    _zz_150_[10] = _zz_149_;
    _zz_150_[9] = _zz_149_;
    _zz_150_[8] = _zz_149_;
    _zz_150_[7] = _zz_149_;
    _zz_150_[6] = _zz_149_;
    _zz_150_[5] = _zz_149_;
    _zz_150_[4] = _zz_149_;
    _zz_150_[3] = _zz_149_;
    _zz_150_[2] = _zz_149_;
    _zz_150_[1] = _zz_149_;
    _zz_150_[0] = _zz_149_;
  end

  assign _zz_151_ = _zz_235_[11];
  always @ (*) begin
    _zz_152_[18] = _zz_151_;
    _zz_152_[17] = _zz_151_;
    _zz_152_[16] = _zz_151_;
    _zz_152_[15] = _zz_151_;
    _zz_152_[14] = _zz_151_;
    _zz_152_[13] = _zz_151_;
    _zz_152_[12] = _zz_151_;
    _zz_152_[11] = _zz_151_;
    _zz_152_[10] = _zz_151_;
    _zz_152_[9] = _zz_151_;
    _zz_152_[8] = _zz_151_;
    _zz_152_[7] = _zz_151_;
    _zz_152_[6] = _zz_151_;
    _zz_152_[5] = _zz_151_;
    _zz_152_[4] = _zz_151_;
    _zz_152_[3] = _zz_151_;
    _zz_152_[2] = _zz_151_;
    _zz_152_[1] = _zz_151_;
    _zz_152_[0] = _zz_151_;
  end

  always @ (*) begin
    case(execute_BRANCH_CTRL)
      `BranchCtrlEnum_defaultEncoding_JAL : begin
        _zz_153_ = {{_zz_148_,{{{execute_INSTRUCTION[31],execute_INSTRUCTION[19 : 12]},execute_INSTRUCTION[20]},execute_INSTRUCTION[30 : 21]}},1'b0};
      end
      `BranchCtrlEnum_defaultEncoding_JALR : begin
        _zz_153_ = {_zz_150_,execute_INSTRUCTION[31 : 20]};
      end
      default : begin
        _zz_153_ = {{_zz_152_,{{{execute_INSTRUCTION[31],execute_INSTRUCTION[7]},execute_INSTRUCTION[30 : 25]},execute_INSTRUCTION[11 : 8]}},1'b0};
      end
    endcase
  end

  assign execute_BranchPlugin_branch_src2 = _zz_153_;
  assign execute_BranchPlugin_branchAdder = (execute_BranchPlugin_branch_src1 + execute_BranchPlugin_branch_src2);
  assign _zz_34_ = {execute_BranchPlugin_branchAdder[31 : 1],(1'b0)};
  assign BranchPlugin_jumpInterface_valid = ((memory_arbitration_isValid && memory_BRANCH_DO) && (! 1'b0));
  assign BranchPlugin_jumpInterface_payload = memory_BRANCH_CALC;
  assign BranchPlugin_branchExceptionPort_valid = ((memory_arbitration_isValid && memory_BRANCH_DO) && BranchPlugin_jumpInterface_payload[1]);
  assign BranchPlugin_branchExceptionPort_payload_code = (4'b0000);
  assign BranchPlugin_branchExceptionPort_payload_badAddr = BranchPlugin_jumpInterface_payload;
  always @ (*) begin
    CsrPlugin_privilege = (2'b11);
    if(CsrPlugin_forceMachineWire)begin
      CsrPlugin_privilege = (2'b11);
    end
  end

  assign CsrPlugin_misa_base = (2'b01);
  assign CsrPlugin_misa_extensions = (26'b00000000000000000001000010);
  assign _zz_154_ = (CsrPlugin_mip_MTIP && CsrPlugin_mie_MTIE);
  assign _zz_155_ = (CsrPlugin_mip_MSIP && CsrPlugin_mie_MSIE);
  assign _zz_156_ = (CsrPlugin_mip_MEIP && CsrPlugin_mie_MEIE);
  assign CsrPlugin_exceptionPortCtrl_exceptionTargetPrivilegeUncapped = (2'b11);
  assign CsrPlugin_exceptionPortCtrl_exceptionTargetPrivilege = ((CsrPlugin_privilege < CsrPlugin_exceptionPortCtrl_exceptionTargetPrivilegeUncapped) ? CsrPlugin_exceptionPortCtrl_exceptionTargetPrivilegeUncapped : CsrPlugin_privilege);
  assign _zz_157_ = {decodeExceptionPort_valid,IBusSimplePlugin_decodeExceptionPort_valid};
  assign _zz_158_ = _zz_236_[0];
  assign _zz_159_ = {BranchPlugin_branchExceptionPort_valid,DBusSimplePlugin_memoryExceptionPort_valid};
  assign _zz_160_ = _zz_238_[0];
  always @ (*) begin
    CsrPlugin_exceptionPortCtrl_exceptionValids_decode = CsrPlugin_exceptionPortCtrl_exceptionValidsRegs_decode;
    if(_zz_170_)begin
      CsrPlugin_exceptionPortCtrl_exceptionValids_decode = 1'b1;
    end
    if(decode_arbitration_isFlushed)begin
      CsrPlugin_exceptionPortCtrl_exceptionValids_decode = 1'b0;
    end
  end

  always @ (*) begin
    CsrPlugin_exceptionPortCtrl_exceptionValids_execute = CsrPlugin_exceptionPortCtrl_exceptionValidsRegs_execute;
    if(CsrPlugin_selfException_valid)begin
      CsrPlugin_exceptionPortCtrl_exceptionValids_execute = 1'b1;
    end
    if(execute_arbitration_isFlushed)begin
      CsrPlugin_exceptionPortCtrl_exceptionValids_execute = 1'b0;
    end
  end

  always @ (*) begin
    CsrPlugin_exceptionPortCtrl_exceptionValids_memory = CsrPlugin_exceptionPortCtrl_exceptionValidsRegs_memory;
    if(_zz_171_)begin
      CsrPlugin_exceptionPortCtrl_exceptionValids_memory = 1'b1;
    end
    if(memory_arbitration_isFlushed)begin
      CsrPlugin_exceptionPortCtrl_exceptionValids_memory = 1'b0;
    end
  end

  always @ (*) begin
    CsrPlugin_exceptionPortCtrl_exceptionValids_writeBack = CsrPlugin_exceptionPortCtrl_exceptionValidsRegs_writeBack;
    if(writeBack_arbitration_isFlushed)begin
      CsrPlugin_exceptionPortCtrl_exceptionValids_writeBack = 1'b0;
    end
  end

  assign CsrPlugin_exceptionPendings_0 = CsrPlugin_exceptionPortCtrl_exceptionValidsRegs_decode;
  assign CsrPlugin_exceptionPendings_1 = CsrPlugin_exceptionPortCtrl_exceptionValidsRegs_execute;
  assign CsrPlugin_exceptionPendings_2 = CsrPlugin_exceptionPortCtrl_exceptionValidsRegs_memory;
  assign CsrPlugin_exceptionPendings_3 = CsrPlugin_exceptionPortCtrl_exceptionValidsRegs_writeBack;
  assign CsrPlugin_exception = (CsrPlugin_exceptionPortCtrl_exceptionValids_writeBack && CsrPlugin_allowException);
  assign CsrPlugin_lastStageWasWfi = 1'b0;
  always @ (*) begin
    CsrPlugin_pipelineLiberator_done = ((! ({writeBack_arbitration_isValid,{memory_arbitration_isValid,execute_arbitration_isValid}} != (3'b000))) && IBusSimplePlugin_pcValids_3);
    if(({CsrPlugin_exceptionPortCtrl_exceptionValidsRegs_writeBack,{CsrPlugin_exceptionPortCtrl_exceptionValidsRegs_memory,CsrPlugin_exceptionPortCtrl_exceptionValidsRegs_execute}} != (3'b000)))begin
      CsrPlugin_pipelineLiberator_done = 1'b0;
    end
    if(CsrPlugin_hadException)begin
      CsrPlugin_pipelineLiberator_done = 1'b0;
    end
  end

  assign CsrPlugin_interruptJump = ((CsrPlugin_interrupt_valid && CsrPlugin_pipelineLiberator_done) && CsrPlugin_allowInterrupts);
  always @ (*) begin
    CsrPlugin_targetPrivilege = CsrPlugin_interrupt_targetPrivilege;
    if(CsrPlugin_hadException)begin
      CsrPlugin_targetPrivilege = CsrPlugin_exceptionPortCtrl_exceptionTargetPrivilege;
    end
  end

  always @ (*) begin
    CsrPlugin_trapCause = CsrPlugin_interrupt_code;
    if(CsrPlugin_hadException)begin
      CsrPlugin_trapCause = CsrPlugin_exceptionPortCtrl_exceptionContext_code;
    end
  end

  always @ (*) begin
    CsrPlugin_xtvec_mode = (2'bxx);
    case(CsrPlugin_targetPrivilege)
      2'b11 : begin
        CsrPlugin_xtvec_mode = CsrPlugin_mtvec_mode;
      end
      default : begin
      end
    endcase
  end

  always @ (*) begin
    CsrPlugin_xtvec_base = (30'bxxxxxxxxxxxxxxxxxxxxxxxxxxxxxx);
    case(CsrPlugin_targetPrivilege)
      2'b11 : begin
        CsrPlugin_xtvec_base = CsrPlugin_mtvec_base;
      end
      default : begin
      end
    endcase
  end

  assign contextSwitching = CsrPlugin_jumpInterface_valid;
  assign _zz_32_ = (! (((decode_INSTRUCTION[14 : 13] == (2'b01)) && (decode_INSTRUCTION[19 : 15] == (5'b00000))) || ((decode_INSTRUCTION[14 : 13] == (2'b11)) && (decode_INSTRUCTION[19 : 15] == (5'b00000)))));
  assign _zz_31_ = (decode_INSTRUCTION[13 : 7] != (7'b0100000));
  assign execute_CsrPlugin_inWfi = 1'b0;
  assign execute_CsrPlugin_blockedBySideEffects = ({writeBack_arbitration_isValid,memory_arbitration_isValid} != (2'b00));
  always @ (*) begin
    execute_CsrPlugin_illegalAccess = 1'b1;
    case(execute_CsrPlugin_csrAddress)
      12'b101111000000 : begin
        execute_CsrPlugin_illegalAccess = 1'b0;
      end
      12'b001100000000 : begin
        execute_CsrPlugin_illegalAccess = 1'b0;
      end
      12'b001101000001 : begin
        execute_CsrPlugin_illegalAccess = 1'b0;
      end
      12'b001100000101 : begin
        if(execute_CSR_WRITE_OPCODE)begin
          execute_CsrPlugin_illegalAccess = 1'b0;
        end
      end
      12'b001101000100 : begin
        execute_CsrPlugin_illegalAccess = 1'b0;
      end
      12'b001101000011 : begin
        if(execute_CSR_READ_OPCODE)begin
          execute_CsrPlugin_illegalAccess = 1'b0;
        end
      end
      12'b111111000000 : begin
        if(execute_CSR_READ_OPCODE)begin
          execute_CsrPlugin_illegalAccess = 1'b0;
        end
      end
      12'b001100000100 : begin
        execute_CsrPlugin_illegalAccess = 1'b0;
      end
      12'b001101000010 : begin
        if(execute_CSR_READ_OPCODE)begin
          execute_CsrPlugin_illegalAccess = 1'b0;
        end
      end
      default : begin
      end
    endcase
    if(_zz_185_)begin
      execute_CsrPlugin_illegalAccess = 1'b1;
    end
    if(((! execute_arbitration_isValid) || (! execute_IS_CSR)))begin
      execute_CsrPlugin_illegalAccess = 1'b0;
    end
  end

  always @ (*) begin
    execute_CsrPlugin_illegalInstruction = 1'b0;
    if((execute_arbitration_isValid && (execute_ENV_CTRL == `EnvCtrlEnum_defaultEncoding_XRET)))begin
      if((CsrPlugin_privilege < execute_INSTRUCTION[29 : 28]))begin
        execute_CsrPlugin_illegalInstruction = 1'b1;
      end
    end
  end

  always @ (*) begin
    CsrPlugin_selfException_valid = 1'b0;
    if(_zz_186_)begin
      CsrPlugin_selfException_valid = 1'b1;
    end
  end

  always @ (*) begin
    CsrPlugin_selfException_payload_code = (4'bxxxx);
    if(_zz_186_)begin
      case(CsrPlugin_privilege)
        2'b00 : begin
          CsrPlugin_selfException_payload_code = (4'b1000);
        end
        default : begin
          CsrPlugin_selfException_payload_code = (4'b1011);
        end
      endcase
    end
  end

  assign CsrPlugin_selfException_payload_badAddr = execute_INSTRUCTION;
  always @ (*) begin
    execute_CsrPlugin_readData = (32'b00000000000000000000000000000000);
    case(execute_CsrPlugin_csrAddress)
      12'b101111000000 : begin
        execute_CsrPlugin_readData[31 : 0] = _zz_161_;
      end
      12'b001100000000 : begin
        execute_CsrPlugin_readData[12 : 11] = CsrPlugin_mstatus_MPP;
        execute_CsrPlugin_readData[7 : 7] = CsrPlugin_mstatus_MPIE;
        execute_CsrPlugin_readData[3 : 3] = CsrPlugin_mstatus_MIE;
      end
      12'b001101000001 : begin
        execute_CsrPlugin_readData[31 : 0] = CsrPlugin_mepc;
      end
      12'b001100000101 : begin
      end
      12'b001101000100 : begin
        execute_CsrPlugin_readData[11 : 11] = CsrPlugin_mip_MEIP;
        execute_CsrPlugin_readData[7 : 7] = CsrPlugin_mip_MTIP;
        execute_CsrPlugin_readData[3 : 3] = CsrPlugin_mip_MSIP;
      end
      12'b001101000011 : begin
        execute_CsrPlugin_readData[31 : 0] = CsrPlugin_mtval;
      end
      12'b111111000000 : begin
        execute_CsrPlugin_readData[31 : 0] = _zz_162_;
      end
      12'b001100000100 : begin
        execute_CsrPlugin_readData[11 : 11] = CsrPlugin_mie_MEIE;
        execute_CsrPlugin_readData[7 : 7] = CsrPlugin_mie_MTIE;
        execute_CsrPlugin_readData[3 : 3] = CsrPlugin_mie_MSIE;
      end
      12'b001101000010 : begin
        execute_CsrPlugin_readData[31 : 31] = CsrPlugin_mcause_interrupt;
        execute_CsrPlugin_readData[3 : 0] = CsrPlugin_mcause_exceptionCode;
      end
      default : begin
      end
    endcase
  end

  always @ (*) begin
    execute_CsrPlugin_writeInstruction = ((execute_arbitration_isValid && execute_IS_CSR) && execute_CSR_WRITE_OPCODE);
    if(_zz_185_)begin
      execute_CsrPlugin_writeInstruction = 1'b0;
    end
  end

  always @ (*) begin
    execute_CsrPlugin_readInstruction = ((execute_arbitration_isValid && execute_IS_CSR) && execute_CSR_READ_OPCODE);
    if(_zz_185_)begin
      execute_CsrPlugin_readInstruction = 1'b0;
    end
  end

  assign execute_CsrPlugin_writeEnable = ((execute_CsrPlugin_writeInstruction && (! execute_CsrPlugin_blockedBySideEffects)) && (! execute_arbitration_isStuckByOthers));
  assign execute_CsrPlugin_readEnable = ((execute_CsrPlugin_readInstruction && (! execute_CsrPlugin_blockedBySideEffects)) && (! execute_arbitration_isStuckByOthers));
  assign execute_CsrPlugin_readToWriteData = execute_CsrPlugin_readData;
  always @ (*) begin
    case(_zz_193_)
      1'b0 : begin
        execute_CsrPlugin_writeData = execute_SRC1;
      end
      default : begin
        execute_CsrPlugin_writeData = (execute_INSTRUCTION[12] ? (execute_CsrPlugin_readToWriteData & (~ execute_SRC1)) : (execute_CsrPlugin_readToWriteData | execute_SRC1));
      end
    endcase
  end

  assign execute_CsrPlugin_csrAddress = execute_INSTRUCTION[31 : 20];
  assign _zz_162_ = (_zz_161_ & externalInterruptArray_regNext);
  assign externalInterrupt = (_zz_162_ != (32'b00000000000000000000000000000000));
  assign _zz_27_ = decode_SRC2_CTRL;
  assign _zz_25_ = _zz_68_;
  assign _zz_45_ = decode_to_execute_SRC2_CTRL;
  assign _zz_24_ = decode_ALU_BITWISE_CTRL;
  assign _zz_22_ = _zz_72_;
  assign _zz_52_ = decode_to_execute_ALU_BITWISE_CTRL;
  assign _zz_21_ = decode_ENV_CTRL;
  assign _zz_18_ = execute_ENV_CTRL;
  assign _zz_16_ = memory_ENV_CTRL;
  assign _zz_19_ = _zz_64_;
  assign _zz_30_ = decode_to_execute_ENV_CTRL;
  assign _zz_29_ = execute_to_memory_ENV_CTRL;
  assign _zz_33_ = memory_to_writeBack_ENV_CTRL;
  assign _zz_14_ = decode_BRANCH_CTRL;
  assign _zz_12_ = _zz_75_;
  assign _zz_35_ = decode_to_execute_BRANCH_CTRL;
  assign _zz_11_ = decode_ALU_CTRL;
  assign _zz_9_ = _zz_65_;
  assign _zz_50_ = decode_to_execute_ALU_CTRL;
  assign _zz_8_ = decode_SHIFT_CTRL;
  assign _zz_5_ = execute_SHIFT_CTRL;
  assign _zz_6_ = _zz_71_;
  assign _zz_40_ = decode_to_execute_SHIFT_CTRL;
  assign _zz_38_ = execute_to_memory_SHIFT_CTRL;
  assign _zz_3_ = decode_SRC1_CTRL;
  assign _zz_1_ = _zz_62_;
  assign _zz_47_ = decode_to_execute_SRC1_CTRL;
  assign decode_arbitration_isFlushed = (({writeBack_arbitration_flushNext,{memory_arbitration_flushNext,execute_arbitration_flushNext}} != (3'b000)) || ({writeBack_arbitration_flushIt,{memory_arbitration_flushIt,{execute_arbitration_flushIt,decode_arbitration_flushIt}}} != (4'b0000)));
  assign execute_arbitration_isFlushed = (({writeBack_arbitration_flushNext,memory_arbitration_flushNext} != (2'b00)) || ({writeBack_arbitration_flushIt,{memory_arbitration_flushIt,execute_arbitration_flushIt}} != (3'b000)));
  assign memory_arbitration_isFlushed = ((writeBack_arbitration_flushNext != (1'b0)) || ({writeBack_arbitration_flushIt,memory_arbitration_flushIt} != (2'b00)));
  assign writeBack_arbitration_isFlushed = (1'b0 || (writeBack_arbitration_flushIt != (1'b0)));
  assign decode_arbitration_isStuckByOthers = (decode_arbitration_haltByOther || (((1'b0 || execute_arbitration_isStuck) || memory_arbitration_isStuck) || writeBack_arbitration_isStuck));
  assign decode_arbitration_isStuck = (decode_arbitration_haltItself || decode_arbitration_isStuckByOthers);
  assign decode_arbitration_isMoving = ((! decode_arbitration_isStuck) && (! decode_arbitration_removeIt));
  assign decode_arbitration_isFiring = ((decode_arbitration_isValid && (! decode_arbitration_isStuck)) && (! decode_arbitration_removeIt));
  assign execute_arbitration_isStuckByOthers = (execute_arbitration_haltByOther || ((1'b0 || memory_arbitration_isStuck) || writeBack_arbitration_isStuck));
  assign execute_arbitration_isStuck = (execute_arbitration_haltItself || execute_arbitration_isStuckByOthers);
  assign execute_arbitration_isMoving = ((! execute_arbitration_isStuck) && (! execute_arbitration_removeIt));
  assign execute_arbitration_isFiring = ((execute_arbitration_isValid && (! execute_arbitration_isStuck)) && (! execute_arbitration_removeIt));
  assign memory_arbitration_isStuckByOthers = (memory_arbitration_haltByOther || (1'b0 || writeBack_arbitration_isStuck));
  assign memory_arbitration_isStuck = (memory_arbitration_haltItself || memory_arbitration_isStuckByOthers);
  assign memory_arbitration_isMoving = ((! memory_arbitration_isStuck) && (! memory_arbitration_removeIt));
  assign memory_arbitration_isFiring = ((memory_arbitration_isValid && (! memory_arbitration_isStuck)) && (! memory_arbitration_removeIt));
  assign writeBack_arbitration_isStuckByOthers = (writeBack_arbitration_haltByOther || 1'b0);
  assign writeBack_arbitration_isStuck = (writeBack_arbitration_haltItself || writeBack_arbitration_isStuckByOthers);
  assign writeBack_arbitration_isMoving = ((! writeBack_arbitration_isStuck) && (! writeBack_arbitration_removeIt));
  assign writeBack_arbitration_isFiring = ((writeBack_arbitration_isValid && (! writeBack_arbitration_isStuck)) && (! writeBack_arbitration_removeIt));
  assign iBus_cmd_ready = ((1'b1 && (! iBus_cmd_m2sPipe_valid)) || iBus_cmd_m2sPipe_ready);
  assign iBus_cmd_m2sPipe_valid = _zz_163_;
  assign iBus_cmd_m2sPipe_payload_pc = _zz_164_;
  assign iBusWishbone_ADR = (iBus_cmd_m2sPipe_payload_pc >>> 2);
  assign iBusWishbone_CTI = (3'b000);
  assign iBusWishbone_BTE = (2'b00);
  assign iBusWishbone_SEL = (4'b1111);
  assign iBusWishbone_WE = 1'b0;
  assign iBusWishbone_DAT_MOSI = (32'bxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxx);
  assign iBusWishbone_CYC = iBus_cmd_m2sPipe_valid;
  assign iBusWishbone_STB = iBus_cmd_m2sPipe_valid;
  assign iBus_cmd_m2sPipe_ready = (iBus_cmd_m2sPipe_valid && iBusWishbone_ACK);
  assign iBus_rsp_valid = (iBusWishbone_CYC && iBusWishbone_ACK);
  assign iBus_rsp_payload_inst = iBusWishbone_DAT_MISO;
  assign iBus_rsp_payload_error = 1'b0;
  assign dBus_cmd_halfPipe_valid = dBus_cmd_halfPipe_regs_valid;
  assign dBus_cmd_halfPipe_payload_wr = dBus_cmd_halfPipe_regs_payload_wr;
  assign dBus_cmd_halfPipe_payload_address = dBus_cmd_halfPipe_regs_payload_address;
  assign dBus_cmd_halfPipe_payload_data = dBus_cmd_halfPipe_regs_payload_data;
  assign dBus_cmd_halfPipe_payload_size = dBus_cmd_halfPipe_regs_payload_size;
  assign dBus_cmd_ready = dBus_cmd_halfPipe_regs_ready;
  assign dBusWishbone_ADR = (dBus_cmd_halfPipe_payload_address >>> 2);
  assign dBusWishbone_CTI = (3'b000);
  assign dBusWishbone_BTE = (2'b00);
  always @ (*) begin
    case(dBus_cmd_halfPipe_payload_size)
      2'b00 : begin
        _zz_165_ = (4'b0001);
      end
      2'b01 : begin
        _zz_165_ = (4'b0011);
      end
      default : begin
        _zz_165_ = (4'b1111);
      end
    endcase
  end

  always @ (*) begin
    dBusWishbone_SEL = _zz_246_[3:0];
    if((! dBus_cmd_halfPipe_payload_wr))begin
      dBusWishbone_SEL = (4'b1111);
    end
  end

  assign dBusWishbone_WE = dBus_cmd_halfPipe_payload_wr;
  assign dBusWishbone_DAT_MOSI = dBus_cmd_halfPipe_payload_data;
  assign dBus_cmd_halfPipe_ready = (dBus_cmd_halfPipe_valid && dBusWishbone_ACK);
  assign dBusWishbone_CYC = dBus_cmd_halfPipe_valid;
  assign dBusWishbone_STB = dBus_cmd_halfPipe_valid;
  assign dBus_rsp_ready = ((dBus_cmd_halfPipe_valid && (! dBusWishbone_WE)) && dBusWishbone_ACK);
  assign dBus_rsp_data = dBusWishbone_DAT_MISO;
  assign dBus_rsp_error = 1'b0;
  always @ (posedge clk) begin
    if(reset) begin
      IBusSimplePlugin_fetchPc_pcReg <= externalResetVector;
      IBusSimplePlugin_fetchPc_booted <= 1'b0;
      IBusSimplePlugin_fetchPc_inc <= 1'b0;
      _zz_104_ <= 1'b0;
      _zz_105_ <= 1'b0;
      IBusSimplePlugin_injector_nextPcCalc_valids_0 <= 1'b0;
      IBusSimplePlugin_injector_nextPcCalc_valids_1 <= 1'b0;
      IBusSimplePlugin_injector_nextPcCalc_valids_2 <= 1'b0;
      IBusSimplePlugin_injector_nextPcCalc_valids_3 <= 1'b0;
      IBusSimplePlugin_injector_nextPcCalc_valids_4 <= 1'b0;
      IBusSimplePlugin_injector_decodeRemoved <= 1'b0;
      IBusSimplePlugin_pendingCmd <= (3'b000);
      IBusSimplePlugin_rspJoin_discardCounter <= (3'b000);
      _zz_129_ <= 1'b1;
      _zz_142_ <= 1'b0;
      CsrPlugin_mstatus_MIE <= 1'b0;
      CsrPlugin_mstatus_MPIE <= 1'b0;
      CsrPlugin_mstatus_MPP <= (2'b11);
      CsrPlugin_mie_MEIE <= 1'b0;
      CsrPlugin_mie_MTIE <= 1'b0;
      CsrPlugin_mie_MSIE <= 1'b0;
      CsrPlugin_exceptionPortCtrl_exceptionValidsRegs_decode <= 1'b0;
      CsrPlugin_exceptionPortCtrl_exceptionValidsRegs_execute <= 1'b0;
      CsrPlugin_exceptionPortCtrl_exceptionValidsRegs_memory <= 1'b0;
      CsrPlugin_exceptionPortCtrl_exceptionValidsRegs_writeBack <= 1'b0;
      CsrPlugin_interrupt_valid <= 1'b0;
      CsrPlugin_hadException <= 1'b0;
      execute_CsrPlugin_wfiWake <= 1'b0;
      _zz_161_ <= (32'b00000000000000000000000000000000);
      execute_arbitration_isValid <= 1'b0;
      memory_arbitration_isValid <= 1'b0;
      writeBack_arbitration_isValid <= 1'b0;
      memory_to_writeBack_REGFILE_WRITE_DATA <= (32'b00000000000000000000000000000000);
      memory_to_writeBack_INSTRUCTION <= (32'b00000000000000000000000000000000);
      _zz_163_ <= 1'b0;
      dBus_cmd_halfPipe_regs_valid <= 1'b0;
      dBus_cmd_halfPipe_regs_ready <= 1'b1;
    end else begin
      IBusSimplePlugin_fetchPc_booted <= 1'b1;
      if((IBusSimplePlugin_fetchPc_corrected || IBusSimplePlugin_fetchPc_pcRegPropagate))begin
        IBusSimplePlugin_fetchPc_inc <= 1'b0;
      end
      if((IBusSimplePlugin_fetchPc_output_valid && IBusSimplePlugin_fetchPc_output_ready))begin
        IBusSimplePlugin_fetchPc_inc <= 1'b1;
      end
      if(((! IBusSimplePlugin_fetchPc_output_valid) && IBusSimplePlugin_fetchPc_output_ready))begin
        IBusSimplePlugin_fetchPc_inc <= 1'b0;
      end
      if((IBusSimplePlugin_fetchPc_booted && ((IBusSimplePlugin_fetchPc_output_ready || IBusSimplePlugin_fetcherflushIt) || IBusSimplePlugin_fetchPc_pcRegPropagate)))begin
        IBusSimplePlugin_fetchPc_pcReg <= IBusSimplePlugin_fetchPc_pc;
      end
      if(IBusSimplePlugin_fetcherflushIt)begin
        _zz_104_ <= 1'b0;
      end
      if(_zz_102_)begin
        _zz_104_ <= IBusSimplePlugin_iBusRsp_stages_0_output_valid;
      end
      if(IBusSimplePlugin_iBusRsp_inputBeforeStage_ready)begin
        _zz_105_ <= IBusSimplePlugin_iBusRsp_inputBeforeStage_valid;
      end
      if(IBusSimplePlugin_fetcherflushIt)begin
        _zz_105_ <= 1'b0;
      end
      if(IBusSimplePlugin_fetcherflushIt)begin
        IBusSimplePlugin_injector_nextPcCalc_valids_0 <= 1'b0;
      end
      if((! (! IBusSimplePlugin_iBusRsp_stages_1_input_ready)))begin
        IBusSimplePlugin_injector_nextPcCalc_valids_0 <= 1'b1;
      end
      if(IBusSimplePlugin_fetcherflushIt)begin
        IBusSimplePlugin_injector_nextPcCalc_valids_1 <= 1'b0;
      end
      if((! (! IBusSimplePlugin_injector_decodeInput_ready)))begin
        IBusSimplePlugin_injector_nextPcCalc_valids_1 <= IBusSimplePlugin_injector_nextPcCalc_valids_0;
      end
      if(IBusSimplePlugin_fetcherflushIt)begin
        IBusSimplePlugin_injector_nextPcCalc_valids_1 <= 1'b0;
      end
      if(IBusSimplePlugin_fetcherflushIt)begin
        IBusSimplePlugin_injector_nextPcCalc_valids_2 <= 1'b0;
      end
      if((! execute_arbitration_isStuck))begin
        IBusSimplePlugin_injector_nextPcCalc_valids_2 <= IBusSimplePlugin_injector_nextPcCalc_valids_1;
      end
      if(IBusSimplePlugin_fetcherflushIt)begin
        IBusSimplePlugin_injector_nextPcCalc_valids_2 <= 1'b0;
      end
      if(IBusSimplePlugin_fetcherflushIt)begin
        IBusSimplePlugin_injector_nextPcCalc_valids_3 <= 1'b0;
      end
      if((! memory_arbitration_isStuck))begin
        IBusSimplePlugin_injector_nextPcCalc_valids_3 <= IBusSimplePlugin_injector_nextPcCalc_valids_2;
      end
      if(IBusSimplePlugin_fetcherflushIt)begin
        IBusSimplePlugin_injector_nextPcCalc_valids_3 <= 1'b0;
      end
      if(IBusSimplePlugin_fetcherflushIt)begin
        IBusSimplePlugin_injector_nextPcCalc_valids_4 <= 1'b0;
      end
      if((! writeBack_arbitration_isStuck))begin
        IBusSimplePlugin_injector_nextPcCalc_valids_4 <= IBusSimplePlugin_injector_nextPcCalc_valids_3;
      end
      if(IBusSimplePlugin_fetcherflushIt)begin
        IBusSimplePlugin_injector_nextPcCalc_valids_4 <= 1'b0;
      end
      if(decode_arbitration_removeIt)begin
        IBusSimplePlugin_injector_decodeRemoved <= 1'b1;
      end
      if(IBusSimplePlugin_fetcherflushIt)begin
        IBusSimplePlugin_injector_decodeRemoved <= 1'b0;
      end
      IBusSimplePlugin_pendingCmd <= IBusSimplePlugin_pendingCmdNext;
      IBusSimplePlugin_rspJoin_discardCounter <= (IBusSimplePlugin_rspJoin_discardCounter - _zz_203_);
      if(IBusSimplePlugin_fetcherflushIt)begin
        IBusSimplePlugin_rspJoin_discardCounter <= (IBusSimplePlugin_pendingCmd - _zz_205_);
      end
      _zz_129_ <= 1'b0;
      _zz_142_ <= _zz_141_;
      if((! decode_arbitration_isStuck))begin
        CsrPlugin_exceptionPortCtrl_exceptionValidsRegs_decode <= 1'b0;
      end else begin
        CsrPlugin_exceptionPortCtrl_exceptionValidsRegs_decode <= CsrPlugin_exceptionPortCtrl_exceptionValids_decode;
      end
      if((! execute_arbitration_isStuck))begin
        CsrPlugin_exceptionPortCtrl_exceptionValidsRegs_execute <= (CsrPlugin_exceptionPortCtrl_exceptionValids_decode && (! decode_arbitration_isStuck));
      end else begin
        CsrPlugin_exceptionPortCtrl_exceptionValidsRegs_execute <= CsrPlugin_exceptionPortCtrl_exceptionValids_execute;
      end
      if((! memory_arbitration_isStuck))begin
        CsrPlugin_exceptionPortCtrl_exceptionValidsRegs_memory <= (CsrPlugin_exceptionPortCtrl_exceptionValids_execute && (! execute_arbitration_isStuck));
      end else begin
        CsrPlugin_exceptionPortCtrl_exceptionValidsRegs_memory <= CsrPlugin_exceptionPortCtrl_exceptionValids_memory;
      end
      if((! writeBack_arbitration_isStuck))begin
        CsrPlugin_exceptionPortCtrl_exceptionValidsRegs_writeBack <= (CsrPlugin_exceptionPortCtrl_exceptionValids_memory && (! memory_arbitration_isStuck));
      end else begin
        CsrPlugin_exceptionPortCtrl_exceptionValidsRegs_writeBack <= 1'b0;
      end
      CsrPlugin_interrupt_valid <= 1'b0;
      if(_zz_187_)begin
        if(_zz_188_)begin
          CsrPlugin_interrupt_valid <= 1'b1;
        end
        if(_zz_189_)begin
          CsrPlugin_interrupt_valid <= 1'b1;
        end
        if(_zz_190_)begin
          CsrPlugin_interrupt_valid <= 1'b1;
        end
      end
      CsrPlugin_hadException <= CsrPlugin_exception;
      if(_zz_172_)begin
        case(CsrPlugin_targetPrivilege)
          2'b11 : begin
            CsrPlugin_mstatus_MIE <= 1'b0;
            CsrPlugin_mstatus_MPIE <= CsrPlugin_mstatus_MIE;
            CsrPlugin_mstatus_MPP <= CsrPlugin_privilege;
          end
          default : begin
          end
        endcase
      end
      if(_zz_173_)begin
        case(_zz_174_)
          2'b11 : begin
            CsrPlugin_mstatus_MPP <= (2'b00);
            CsrPlugin_mstatus_MIE <= CsrPlugin_mstatus_MPIE;
            CsrPlugin_mstatus_MPIE <= 1'b1;
          end
          default : begin
          end
        endcase
      end
      execute_CsrPlugin_wfiWake <= ({_zz_156_,{_zz_155_,_zz_154_}} != (3'b000));
      if((! writeBack_arbitration_isStuck))begin
        memory_to_writeBack_REGFILE_WRITE_DATA <= _zz_37_;
      end
      if((! writeBack_arbitration_isStuck))begin
        memory_to_writeBack_INSTRUCTION <= memory_INSTRUCTION;
      end
      if(((! execute_arbitration_isStuck) || execute_arbitration_removeIt))begin
        execute_arbitration_isValid <= 1'b0;
      end
      if(((! decode_arbitration_isStuck) && (! decode_arbitration_removeIt)))begin
        execute_arbitration_isValid <= decode_arbitration_isValid;
      end
      if(((! memory_arbitration_isStuck) || memory_arbitration_removeIt))begin
        memory_arbitration_isValid <= 1'b0;
      end
      if(((! execute_arbitration_isStuck) && (! execute_arbitration_removeIt)))begin
        memory_arbitration_isValid <= execute_arbitration_isValid;
      end
      if(((! writeBack_arbitration_isStuck) || writeBack_arbitration_removeIt))begin
        writeBack_arbitration_isValid <= 1'b0;
      end
      if(((! memory_arbitration_isStuck) && (! memory_arbitration_removeIt)))begin
        writeBack_arbitration_isValid <= memory_arbitration_isValid;
      end
      case(execute_CsrPlugin_csrAddress)
        12'b101111000000 : begin
          if(execute_CsrPlugin_writeEnable)begin
            _zz_161_ <= execute_CsrPlugin_writeData[31 : 0];
          end
        end
        12'b001100000000 : begin
          if(execute_CsrPlugin_writeEnable)begin
            CsrPlugin_mstatus_MPP <= execute_CsrPlugin_writeData[12 : 11];
            CsrPlugin_mstatus_MPIE <= _zz_240_[0];
            CsrPlugin_mstatus_MIE <= _zz_241_[0];
          end
        end
        12'b001101000001 : begin
        end
        12'b001100000101 : begin
        end
        12'b001101000100 : begin
        end
        12'b001101000011 : begin
        end
        12'b111111000000 : begin
        end
        12'b001100000100 : begin
          if(execute_CsrPlugin_writeEnable)begin
            CsrPlugin_mie_MEIE <= _zz_243_[0];
            CsrPlugin_mie_MTIE <= _zz_244_[0];
            CsrPlugin_mie_MSIE <= _zz_245_[0];
          end
        end
        12'b001101000010 : begin
        end
        default : begin
        end
      endcase
      if(iBus_cmd_ready)begin
        _zz_163_ <= iBus_cmd_valid;
      end
      if(_zz_191_)begin
        dBus_cmd_halfPipe_regs_valid <= dBus_cmd_valid;
        dBus_cmd_halfPipe_regs_ready <= (! dBus_cmd_valid);
      end else begin
        dBus_cmd_halfPipe_regs_valid <= (! dBus_cmd_halfPipe_ready);
        dBus_cmd_halfPipe_regs_ready <= dBus_cmd_halfPipe_ready;
      end
    end
  end

  always @ (posedge clk) begin
    if(IBusSimplePlugin_iBusRsp_inputBeforeStage_ready)begin
      _zz_106_ <= IBusSimplePlugin_iBusRsp_inputBeforeStage_payload_pc;
      _zz_107_ <= IBusSimplePlugin_iBusRsp_inputBeforeStage_payload_rsp_error;
      _zz_108_ <= IBusSimplePlugin_iBusRsp_inputBeforeStage_payload_rsp_inst;
      _zz_109_ <= IBusSimplePlugin_iBusRsp_inputBeforeStage_payload_isRvc;
    end
    if(IBusSimplePlugin_injector_decodeInput_ready)begin
      IBusSimplePlugin_injector_formal_rawInDecode <= IBusSimplePlugin_iBusRsp_inputBeforeStage_payload_rsp_inst;
    end
    if(IBusSimplePlugin_iBusRsp_stages_1_output_ready)begin
      IBusSimplePlugin_mmu_joinCtx_physicalAddress <= IBusSimplePlugin_mmuBus_rsp_physicalAddress;
      IBusSimplePlugin_mmu_joinCtx_isIoAccess <= IBusSimplePlugin_mmuBus_rsp_isIoAccess;
      IBusSimplePlugin_mmu_joinCtx_allowRead <= IBusSimplePlugin_mmuBus_rsp_allowRead;
      IBusSimplePlugin_mmu_joinCtx_allowWrite <= IBusSimplePlugin_mmuBus_rsp_allowWrite;
      IBusSimplePlugin_mmu_joinCtx_allowExecute <= IBusSimplePlugin_mmuBus_rsp_allowExecute;
      IBusSimplePlugin_mmu_joinCtx_exception <= IBusSimplePlugin_mmuBus_rsp_exception;
      IBusSimplePlugin_mmu_joinCtx_refilling <= IBusSimplePlugin_mmuBus_rsp_refilling;
    end
    if(!(! (((dBus_rsp_ready && memory_MEMORY_ENABLE) && memory_arbitration_isValid) && memory_arbitration_isStuck))) begin
      $display("ERROR DBusSimplePlugin doesn't allow memory stage stall when read happend");
    end
    if(!(! (((writeBack_arbitration_isValid && writeBack_MEMORY_ENABLE) && (! writeBack_MEMORY_STORE)) && writeBack_arbitration_isStuck))) begin
      $display("ERROR DBusSimplePlugin doesn't allow writeback stage stall when read happend");
    end
    if(_zz_141_)begin
      _zz_143_ <= _zz_53_[11 : 7];
    end
    CsrPlugin_mip_MEIP <= externalInterrupt;
    CsrPlugin_mip_MTIP <= timerInterrupt;
    CsrPlugin_mip_MSIP <= softwareInterrupt;
    CsrPlugin_mcycle <= (CsrPlugin_mcycle + (64'b0000000000000000000000000000000000000000000000000000000000000001));
    if(writeBack_arbitration_isFiring)begin
      CsrPlugin_minstret <= (CsrPlugin_minstret + (64'b0000000000000000000000000000000000000000000000000000000000000001));
    end
    if(_zz_170_)begin
      CsrPlugin_exceptionPortCtrl_exceptionContext_code <= (_zz_158_ ? IBusSimplePlugin_decodeExceptionPort_payload_code : decodeExceptionPort_payload_code);
      CsrPlugin_exceptionPortCtrl_exceptionContext_badAddr <= (_zz_158_ ? IBusSimplePlugin_decodeExceptionPort_payload_badAddr : decodeExceptionPort_payload_badAddr);
    end
    if(CsrPlugin_selfException_valid)begin
      CsrPlugin_exceptionPortCtrl_exceptionContext_code <= CsrPlugin_selfException_payload_code;
      CsrPlugin_exceptionPortCtrl_exceptionContext_badAddr <= CsrPlugin_selfException_payload_badAddr;
    end
    if(_zz_171_)begin
      CsrPlugin_exceptionPortCtrl_exceptionContext_code <= (_zz_160_ ? DBusSimplePlugin_memoryExceptionPort_payload_code : BranchPlugin_branchExceptionPort_payload_code);
      CsrPlugin_exceptionPortCtrl_exceptionContext_badAddr <= (_zz_160_ ? DBusSimplePlugin_memoryExceptionPort_payload_badAddr : BranchPlugin_branchExceptionPort_payload_badAddr);
    end
    if(_zz_187_)begin
      if(_zz_188_)begin
        CsrPlugin_interrupt_code <= (4'b0111);
        CsrPlugin_interrupt_targetPrivilege <= (2'b11);
      end
      if(_zz_189_)begin
        CsrPlugin_interrupt_code <= (4'b0011);
        CsrPlugin_interrupt_targetPrivilege <= (2'b11);
      end
      if(_zz_190_)begin
        CsrPlugin_interrupt_code <= (4'b1011);
        CsrPlugin_interrupt_targetPrivilege <= (2'b11);
      end
    end
    if(_zz_172_)begin
      case(CsrPlugin_targetPrivilege)
        2'b11 : begin
          CsrPlugin_mcause_interrupt <= (! CsrPlugin_hadException);
          CsrPlugin_mcause_exceptionCode <= CsrPlugin_trapCause;
          CsrPlugin_mepc <= writeBack_PC;
          if(CsrPlugin_hadException)begin
            CsrPlugin_mtval <= CsrPlugin_exceptionPortCtrl_exceptionContext_badAddr;
          end
        end
        default : begin
        end
      endcase
    end
    externalInterruptArray_regNext <= externalInterruptArray;
    if((! memory_arbitration_isStuck))begin
      execute_to_memory_ALIGNEMENT_FAULT <= execute_ALIGNEMENT_FAULT;
    end
    if((! execute_arbitration_isStuck))begin
      decode_to_execute_SRC2_CTRL <= _zz_26_;
    end
    if((! execute_arbitration_isStuck))begin
      decode_to_execute_MEMORY_ENABLE <= decode_MEMORY_ENABLE;
    end
    if((! memory_arbitration_isStuck))begin
      execute_to_memory_MEMORY_ENABLE <= execute_MEMORY_ENABLE;
    end
    if((! writeBack_arbitration_isStuck))begin
      memory_to_writeBack_MEMORY_ENABLE <= memory_MEMORY_ENABLE;
    end
    if((! execute_arbitration_isStuck))begin
      decode_to_execute_ALU_BITWISE_CTRL <= _zz_23_;
    end
    if((! execute_arbitration_isStuck))begin
      decode_to_execute_BYPASSABLE_MEMORY_STAGE <= decode_BYPASSABLE_MEMORY_STAGE;
    end
    if((! memory_arbitration_isStuck))begin
      execute_to_memory_BYPASSABLE_MEMORY_STAGE <= execute_BYPASSABLE_MEMORY_STAGE;
    end
    if((! execute_arbitration_isStuck))begin
      decode_to_execute_RS1 <= decode_RS1;
    end
    if((! execute_arbitration_isStuck))begin
      decode_to_execute_SRC_USE_SUB_LESS <= decode_SRC_USE_SUB_LESS;
    end
    if((! memory_arbitration_isStuck))begin
      execute_to_memory_BRANCH_DO <= execute_BRANCH_DO;
    end
    if((! execute_arbitration_isStuck))begin
      decode_to_execute_MEMORY_STORE <= decode_MEMORY_STORE;
    end
    if((! memory_arbitration_isStuck))begin
      execute_to_memory_MEMORY_STORE <= execute_MEMORY_STORE;
    end
    if((! writeBack_arbitration_isStuck))begin
      memory_to_writeBack_MEMORY_STORE <= memory_MEMORY_STORE;
    end
    if((! memory_arbitration_isStuck))begin
      execute_to_memory_BRANCH_CALC <= execute_BRANCH_CALC;
    end
    if((! execute_arbitration_isStuck))begin
      decode_to_execute_REGFILE_WRITE_VALID <= decode_REGFILE_WRITE_VALID;
    end
    if((! memory_arbitration_isStuck))begin
      execute_to_memory_REGFILE_WRITE_VALID <= execute_REGFILE_WRITE_VALID;
    end
    if((! writeBack_arbitration_isStuck))begin
      memory_to_writeBack_REGFILE_WRITE_VALID <= memory_REGFILE_WRITE_VALID;
    end
    if((! execute_arbitration_isStuck))begin
      decode_to_execute_RS2 <= decode_RS2;
    end
    if((! memory_arbitration_isStuck))begin
      execute_to_memory_MMU_FAULT <= execute_MMU_FAULT;
    end
    if((! memory_arbitration_isStuck))begin
      execute_to_memory_SHIFT_RIGHT <= execute_SHIFT_RIGHT;
    end
    if((! memory_arbitration_isStuck))begin
      execute_to_memory_REGFILE_WRITE_DATA <= _zz_28_;
    end
    if((! execute_arbitration_isStuck))begin
      decode_to_execute_SRC_LESS_UNSIGNED <= decode_SRC_LESS_UNSIGNED;
    end
    if((! execute_arbitration_isStuck))begin
      decode_to_execute_BYPASSABLE_EXECUTE_STAGE <= decode_BYPASSABLE_EXECUTE_STAGE;
    end
    if((! memory_arbitration_isStuck))begin
      execute_to_memory_MEMORY_ADDRESS_LOW <= execute_MEMORY_ADDRESS_LOW;
    end
    if((! writeBack_arbitration_isStuck))begin
      memory_to_writeBack_MEMORY_ADDRESS_LOW <= memory_MEMORY_ADDRESS_LOW;
    end
    if((! execute_arbitration_isStuck))begin
      decode_to_execute_CSR_READ_OPCODE <= decode_CSR_READ_OPCODE;
    end
    if((! execute_arbitration_isStuck))begin
      decode_to_execute_ENV_CTRL <= _zz_20_;
    end
    if((! memory_arbitration_isStuck))begin
      execute_to_memory_ENV_CTRL <= _zz_17_;
    end
    if((! writeBack_arbitration_isStuck))begin
      memory_to_writeBack_ENV_CTRL <= _zz_15_;
    end
    if((! execute_arbitration_isStuck))begin
      decode_to_execute_FORMAL_PC_NEXT <= _zz_90_;
    end
    if((! memory_arbitration_isStuck))begin
      execute_to_memory_FORMAL_PC_NEXT <= execute_FORMAL_PC_NEXT;
    end
    if((! writeBack_arbitration_isStuck))begin
      memory_to_writeBack_FORMAL_PC_NEXT <= _zz_89_;
    end
    if((! execute_arbitration_isStuck))begin
      decode_to_execute_PC <= decode_PC;
    end
    if((! memory_arbitration_isStuck))begin
      execute_to_memory_PC <= _zz_44_;
    end
    if(((! writeBack_arbitration_isStuck) && (! CsrPlugin_exceptionPortCtrl_exceptionValids_writeBack)))begin
      memory_to_writeBack_PC <= memory_PC;
    end
    if((! execute_arbitration_isStuck))begin
      decode_to_execute_IS_CSR <= decode_IS_CSR;
    end
    if((! memory_arbitration_isStuck))begin
      execute_to_memory_MMU_RSP_physicalAddress <= execute_MMU_RSP_physicalAddress;
      execute_to_memory_MMU_RSP_isIoAccess <= execute_MMU_RSP_isIoAccess;
      execute_to_memory_MMU_RSP_allowRead <= execute_MMU_RSP_allowRead;
      execute_to_memory_MMU_RSP_allowWrite <= execute_MMU_RSP_allowWrite;
      execute_to_memory_MMU_RSP_allowExecute <= execute_MMU_RSP_allowExecute;
      execute_to_memory_MMU_RSP_exception <= execute_MMU_RSP_exception;
      execute_to_memory_MMU_RSP_refilling <= execute_MMU_RSP_refilling;
    end
    if((! execute_arbitration_isStuck))begin
      decode_to_execute_BRANCH_CTRL <= _zz_13_;
    end
    if((! execute_arbitration_isStuck))begin
      decode_to_execute_ALU_CTRL <= _zz_10_;
    end
    if((! execute_arbitration_isStuck))begin
      decode_to_execute_SRC2_FORCE_ZERO <= decode_SRC2_FORCE_ZERO;
    end
    if((! execute_arbitration_isStuck))begin
      decode_to_execute_SHIFT_CTRL <= _zz_7_;
    end
    if((! memory_arbitration_isStuck))begin
      execute_to_memory_SHIFT_CTRL <= _zz_4_;
    end
    if((! writeBack_arbitration_isStuck))begin
      memory_to_writeBack_MEMORY_READ_DATA <= memory_MEMORY_READ_DATA;
    end
    if((! execute_arbitration_isStuck))begin
      decode_to_execute_CSR_WRITE_OPCODE <= decode_CSR_WRITE_OPCODE;
    end
    if((! execute_arbitration_isStuck))begin
      decode_to_execute_SRC1_CTRL <= _zz_2_;
    end
    if((! execute_arbitration_isStuck))begin
      decode_to_execute_INSTRUCTION <= decode_INSTRUCTION;
    end
    if((! memory_arbitration_isStuck))begin
      execute_to_memory_INSTRUCTION <= execute_INSTRUCTION;
    end
    case(execute_CsrPlugin_csrAddress)
      12'b101111000000 : begin
      end
      12'b001100000000 : begin
      end
      12'b001101000001 : begin
        if(execute_CsrPlugin_writeEnable)begin
          CsrPlugin_mepc <= execute_CsrPlugin_writeData[31 : 0];
        end
      end
      12'b001100000101 : begin
        if(execute_CsrPlugin_writeEnable)begin
          CsrPlugin_mtvec_base <= execute_CsrPlugin_writeData[31 : 2];
          CsrPlugin_mtvec_mode <= execute_CsrPlugin_writeData[1 : 0];
        end
      end
      12'b001101000100 : begin
        if(execute_CsrPlugin_writeEnable)begin
          CsrPlugin_mip_MSIP <= _zz_242_[0];
        end
      end
      12'b001101000011 : begin
      end
      12'b111111000000 : begin
      end
      12'b001100000100 : begin
      end
      12'b001101000010 : begin
      end
      default : begin
      end
    endcase
    if(iBus_cmd_ready)begin
      _zz_164_ <= iBus_cmd_payload_pc;
    end
    if(_zz_191_)begin
      dBus_cmd_halfPipe_regs_payload_wr <= dBus_cmd_payload_wr;
      dBus_cmd_halfPipe_regs_payload_address <= dBus_cmd_payload_address;
      dBus_cmd_halfPipe_regs_payload_data <= dBus_cmd_payload_data;
      dBus_cmd_halfPipe_regs_payload_size <= dBus_cmd_payload_size;
    end
  end

endmodule

