

module MyMiniGpu (
    input  wire        clk,
    input  wire        rst_n,
    input  wire        start,
    input  wire [7:0]  ui_in,      // Fysiska ingångar
    output wire [7:0]  uo_out,     // Fysiska utgångar
    output wire [7:0]  uio_out,    // LÄGG TILL DENNA PORT!
    output wire [7:0]  uio_oe,     // LÄGG TILL DENNA PORT!
    output wire        done
);
    wire [15:0] dispatcher_current_instruction;
        wire [15:0] ram_rdata;
    // Interna kontrollbussar
    wire [1:0]  alu_op, raddr_a, raddr_b, waddr;
    wire [3:0]  pe_mask;
    wire        reg_write, load_en, io_ready, io_done_pulse; // <--- Deklarerad io_done_pulse
    wire [31:0] broad_din;
    wire [31:0] broad_dout;
    // 1. Instansiera Kontrollenheten (Dispatcher)
    GpuDispatcher #( .NUM_PES(4) ) dispatcher_inst (
        .clk(clk),
        .rst_n(rst_n),
        .start(start),
        .io_ready(io_ready),       
        .io_done_pulse(io_done_pulse), // <--- Koppla in den här!
        .out_alu_op(alu_op),
        .out_raddr_a(raddr_a),
        .out_raddr_b(raddr_b),
        .out_waddr(waddr),
        .out_pe_mask(pe_mask),
        .out_reg_write(reg_write),
        .out_load_en(load_en),    
        .current_instruction(ram_rdata),
        .out_current_instruction(dispatcher_current_instruction), 
        .out_done(done),
        .out_pc(gpu_pc)

    );

    // Inuti din toppmodul (MyMiniGpu / tt_um_...):
    
    wire req_store;
    // Dispatcher skickar ut opcode 2'b10 när det är en STORE-instruktion
    assign req_store = (dispatcher_current_instruction[15:14] == 2'b10) && io_ready;

    // 2. Uppdatera I/O-gränssnittet i toppmodulen så att req_store är inkopplad!
    GpuIoInterface #( .NUM_PES(4), .DATA_WIDTH(8) ) io_inst (
        .clk(clk),
        .rst_n(rst_n),
        .io_din(ui_in),
        .io_dout(uo_out),       // De fysiska utpinnarna styrs nu av I/O-modulen!
        .req_load(load_en),        
        .req_store(req_store),  // <--- ÄNDRA HÄR (från 1'b0 till req_store)
        .io_ready(io_ready),       
        .io_done_pulse(io_done_pulse), 
        .gpu_broad_din(broad_din),
        .gpu_broad_dout(broad_dout), // <--- Resultaten från kärnorna strömmar nu hit!
        .iram_we(iram_we),
.iram_waddr(iram_waddr),
.iram_wdata(iram_wdata)

    );


    // 3. Instansiera beräkningsmatrisen (SIMD-kärnorna)
    SimdCore #( .NUM_PES(4), .DATA_WIDTH(8) ) simd_inst (
        .clk(clk),
        .rst_n(rst_n),
        .inst_alu_op(alu_op),
        .inst_raddr_a(raddr_a),
        .inst_raddr_b(raddr_b),
        .inst_waddr(waddr),
        .inst_reg_write(reg_write && io_ready), 
        .inst_pe_mask(pe_mask),
        .broad_din(broad_din),
        .broad_load_en(load_en && io_ready),    
        .broad_dout(broad_dout)
    );

    // Inuti din officiella tt_um_... modul:
    wire [7:0] gpu_output_signals;

    MyMiniGpu min_gpu_inst (
        .clk(clk),
        .rst_n(rst_n),
        .ui_in(ui_in),
        .uo_out(gpu_output_signals), // Fånga upp GPU:ns utsignal
        .start(ui_in[0]),            // Exempel: styrs av ingångs-pinne 0
        .done(uo_out[7])             // Exempel: done-flagga ut på pinne 7
    );

    // DUBBELSÄKRA DENNA RAD: Tvinga utsignalerna till chippets fysiska pinnar!
    assign uo_out = gpu_output_signals; 
    
    // Tiny Tapeout kräver också att oanvända dubbelriktade pinnar tilldelas något
    assign uio_out = 8'b00000000; // Sätter alla dubbelriktade pinnar till 0
    assign uio_oe  = 8'b00000000; // Sätter dem till ingångar (deaktiverade utgångar)

    wire        iram_we;
    wire [15:0] iram_wdata;
        wire [3:0] iram_waddr;
    wire [3:0] gpu_pc;

    wire [15:0] ram_rdata;

    GpuInstructionRam iram_inst (
        .clk(clk),
        // Skrivport (Styrs av mikrokontrollern via I/O-gränssnittet)
        .we(iram_we),
        .waddr(iram_waddr),
        .wdata(iram_wdata),
        
        // Läsport (Styrs av din GPU-kontrollenhet)
        .raddr(gpu_pc),
        .rdata(ram_rdata)
    );
endmodule




module GpuIoInterface #(
    parameter NUM_PES = 4,
    parameter DATA_WIDTH = 8
)(
    input  wire        clk,
    input  wire        rst_n,
    
    // Gränssnitt mot mikrokontrollern
    input  wire [7:0]  io_din,         
    output reg  [7:0]  io_dout,        
    input  wire        prog_mode,      // 1: Mikrokontrollern laddar program, 0: GPU-läge
    input  wire        prog_write_en,  // Pulsas hög av MCU för att skicka en byte
    
    // Gränssnitt mot det interna Instruktions-RAM-minnet
    output reg         iram_we,
    output reg  [4:0]  iram_waddr,
    output reg  [15:0] iram_wdata,
    
    // Interna GPU-signaler (LOAD/STORE)
    input  wire        req_load,       
    input  wire        req_store,      
    output wire        io_ready,       
    output reg         io_done_pulse,  
    output wire [DATA_WIDTH*NUM_PES-1:0] gpu_broad_din,
    input  wire [DATA_WIDTH*NUM_PES-1:0] gpu_broad_dout
);

    reg [1:0] stream_cnt;
    reg [1:0] state;
    reg       io_active;
    
    // Nya register för att hantera programnedladdning
    reg       byte_sel; // 0: Övre byten, 1: Nedre byten

    parameter STATE_IDLE  = 2'b00;
    parameter STATE_LOAD  = 2'b01;
    parameter STATE_STORE = 2'b10;

    reg [DATA_WIDTH*NUM_PES-1:0] load_buffer;
    reg [DATA_WIDTH*NUM_PES-1:0] store_buffer;

    assign gpu_broad_din = load_buffer;
    assign io_ready = (state == STATE_IDLE) && !io_active && !prog_mode;

    // Logik för att ladda upp program från mikrokontrollern
    always @(posedge clk or negedge rst_n) begin
        if (!rst_n) begin
            iram_we    <= 1'b0;
            iram_waddr <= 5'd0;
            iram_wdata <= 16'd0;
            byte_sel   <= 1'b0;
        end else if (prog_mode) begin
            iram_we <= 1'b0;
            if (prog_write_en) begin
                if (byte_sel == 1'b0) begin
                    iram_wdata[15:8] <= io_din; // Spara övre halvan av instruktionen
                    byte_sel         <= 1'b1;
                end else begin
                    iram_wdata[7:0] <= io_din;  // Spara nedre halvan
                    iram_we         <= 1'b1;    // Trigga skrivning till RAM
                    byte_sel        <= 1'b0;
                end
            end
            
            // Stega adressen efter avklarad 16-bitars instruktion
            if (iram_we) begin
                iram_waddr <= iram_waddr + 1'b1;
            end
        end else begin
            iram_waddr <= 5'd0; // Nollställ när vi lämnar prog_mode
            iram_we    <= 1'b0;
        end
    end

    // ... (resten av ditt always @(posedge clk) block med STATE_LOAD och STATE_STORE är exakt som förut!) ...

endmodule





// 1. En enskild beräkningskärna (Processing Element)
module GpuDispatcher #(
    
    parameter NUM_PES = 4
)(


    input  wire        clk,
    input  wire        rst_n,
    input  wire        start,          
    input  wire        io_ready,       
    input  wire        io_done_pulse,  
    
    // NYTT GRÄNSSNITT MOT RAM-MINNET
    output wire [4:0]  out_pc,         // Skickas till RAM-minnets raddr
    input  wire [15:0] current_instruction, // Läses in från RAM-minnets rdata
    output wire [15:0] out_current_instruction,    
    
    output reg  [1:0]  out_alu_op,
    output reg  [1:0]  out_raddr_a,
    output reg  [1:0]  out_raddr_b,
    output reg  [1:0]  out_waddr,
    output reg  [NUM_PES-1:0] out_pe_mask, 
    output reg         out_reg_write,  
    output reg         out_load_en,    
    output reg         out_done        
);

    reg [4:0] pc; // Ändrad till 5 bitar för att matcha 32 instruktioner
    reg       running;

    assign out_pc = pc;

    // ... (resten av always @(posedge clk) och always @(*) är exakt som förut!) ...
    assign out_current_instruction = current_instruction;
endmodule


module ProcessingElement #(
    parameter DATA_WIDTH = 8,
    parameter REG_ADDR_WIDTH = 2,
    parameter NUM_PES = 4,
    parameter PE_ID = 0
)(
    input  wire                   clk,
    input  wire                   rst_n,
    input  wire [1:0]             alu_op,      
    input  wire [REG_ADDR_WIDTH-1:0] raddr_a,  
    input  wire [REG_ADDR_WIDTH-1:0] raddr_b,  
    input  wire [REG_ADDR_WIDTH-1:0] waddr,    
    input  wire                   reg_write,   
    input  wire [DATA_WIDTH-1:0]  vector_din,  
    input  wire                   load_en,     
    input  wire [NUM_PES-1:0]     pe_mask,     
    output wire [DATA_WIDTH-1:0]  vector_dout  
);

    reg [DATA_WIDTH-1:0] r0, r1, r2, r3;
    
    wire my_mask_bit = pe_mask[PE_ID]; 
    reg [DATA_WIDTH-1:0] reg_a;
    reg [DATA_WIDTH-1:0] reg_b;
    reg [DATA_WIDTH-1:0] alu_out;

    // Multiplexer för Källregister A
    always @(*) begin
        case (raddr_a)
            2'b00:   reg_a = r0;
            2'b01:   reg_a = r1;
            2'b10:   reg_a = r2;
            2'b11:   reg_a = r3;
            default: reg_a = {DATA_WIDTH{1'b0}};
        endcase
    end

    // Multiplexer för Källregister B
    always @(*) begin
        case (raddr_b)
            2'b00:   reg_b = r0;
            2'b01:   reg_b = r1;
            2'b10:   reg_b = r2;
            2'b11:   reg_b = r3;
            default: reg_b = {DATA_WIDTH{1'b0}};
        endcase
    end

    // ALU-logik (Hanterar nu bara ADD, SUB och MUL kombinatoriskt)
    always @(*) begin
        case (alu_op)
            2'b00:   alu_out = reg_a + reg_b; // ADD
            2'b01:   alu_out = reg_a - reg_b; // SUB
            2'b10:   alu_out = reg_a * reg_b; // MUL
            2'b11:   alu_out = reg_a * reg_b; // För MAC gör vi multiplikationen här först
            default: alu_out = {DATA_WIDTH{1'b0}};
        endcase
    end

    // Helt klockad skrivlogik (Bryter den kombinatoriska loopen)
    always @(posedge clk or negedge rst_n) begin
        if (!rst_n) begin
            r0 <= {DATA_WIDTH{1'b0}};
            r1 <= {DATA_WIDTH{1'b0}};
            r2 <= {DATA_WIDTH{1'b0}};
            r3 <= {DATA_WIDTH{1'b0}};
        end else if (my_mask_bit) begin
            if (load_en) begin
                case (waddr)
                    2'b00: r0 <= vector_din;
                    2'b01: r1 <= vector_din;
                    2'b10: r2 <= vector_din;
                    2'b11: r3 <= vector_din;
                endcase
            end else if (reg_write) begin
                // Om alu_op är 2'b11 gör vi en MAC (Gamla värdet + nya multiplikationsresultatet)
                if (alu_op == 2'b11) begin
                    case (waddr)
                        2'b00: r0 <= r0 + alu_out;
                        2'b01: r1 <= r1 + alu_out;
                        2'b10: r2 <= r2 + alu_out;
                        2'b11: r3 <= r3 + alu_out;
                    endcase
                end else begin
                    // Vanlig ADD/SUB/MUL sparar bara resultatet direkt
                    case (waddr)
                        2'b00: r0 <= alu_out;
                        2'b01: r1 <= alu_out;
                        2'b10: r2 <= alu_out;
                        2'b11: r3 <= alu_out;
                    endcase
                end
            end
        end
    end

    assign vector_dout = reg_a;

endmodule




// 2. Toppmodulen som binder ihop 4 parallella kärnor (SIMD)
module SimdCore #(
    parameter NUM_PES = 4,
    parameter DATA_WIDTH = 8
)(
    input  wire        clk,
    input  wire        rst_n,
    input  wire [1:0]  inst_alu_op,
    input  wire [1:0]  inst_raddr_a,
    input  wire [1:0]  inst_raddr_b,
    input  wire [1:0]  inst_waddr,
    input  wire        inst_reg_write,
    input  wire [NUM_PES-1:0] inst_pe_mask,
    
    input  wire [DATA_WIDTH*NUM_PES-1:0] broad_din,
    input  wire                          broad_load_en,
    output wire [DATA_WIDTH*NUM_PES-1:0] broad_dout
);

    generate
        genvar i; // Säkerställ att den är deklarerad lokalt inuti generatblocket
        for (i = 0; i < NUM_PES; i = i + 1) begin : PE_GEN
            ProcessingElement #(
                .DATA_WIDTH(DATA_WIDTH),
                .REG_ADDR_WIDTH(2),
                .NUM_PES(NUM_PES),
                .PE_ID(i)
            ) pe_inst (
                .clk(clk),
                .rst_n(rst_n),
                .alu_op(inst_alu_op),
                .raddr_a(inst_raddr_a),
                .raddr_b(inst_raddr_b),
                .waddr(inst_waddr),
                .reg_write(inst_reg_write),
                .pe_mask(inst_pe_mask),
                
                // NY ROBUST SYNTAX: Starta på (i * 8) och ta 8 bitar uppåt (+: 8)
                .vector_din(broad_din[i*DATA_WIDTH +: DATA_WIDTH]),
                .load_en(broad_load_en),
                .vector_dout(broad_dout[i*DATA_WIDTH +: DATA_WIDTH])
            );
        end
    endgenerate

endmodule


module GpuInstructionRam (
    input  wire        clk,
    input  wire        we,             // Write Enable
    input  wire [3:0]  waddr,          // Ändrad till 4 bitar (16 instruktioner)
    input  wire [15:0] wdata,          
    
    input  wire [3:0]  raddr,          // Ändrad till 4 bitar (pc)
    output reg  [15:0] rdata           
);

    // 16 plattade instruktionsregister (Yosys älskar detta!)
    reg [15:0] inst0, inst1, inst2, inst3, inst4, inst5, inst6, inst7;
    reg [15:0] inst8, inst9, inst10, inst11, inst12, inst13, inst14, inst15;

    // Synkron skrivning från mikrokontrollern
    always @(posedge clk) begin
        if (we) begin
            case (waddr)
                4'd0:  inst0  <= wdata;
                4'd1:  inst1  <= wdata;
                4'd2:  inst2  <= wdata;
                4'd3:  inst3  <= wdata;
                4'd4:  inst4  <= wdata;
                4'd5:  inst5  <= wdata;
                4'd6:  inst6  <= wdata;
                4'd7:  inst7  <= wdata;
                4'd8:  inst8  <= wdata;
                4'd9:  inst9  <= wdata;
                4'd10: inst10 <= wdata;
                4'd11: inst11 <= wdata;
                4'd12: inst12 <= wdata;
                4'd13: inst13 <= wdata;
                4'd14: inst14 <= wdata;
                4'd15: inst15 <= wdata;
            endcase
        end
    end

    // Kombinatorisk läsning för GPU:ns Program Counter
    always @(*) begin
        case (raddr)
            4'd0:  rdata = inst0;
            4'd1:  rdata = inst1;
            4'd2:  rdata = inst2;
            4'd3:  rdata = inst3;
            4'd4:  rdata = inst4;
            4'd5:  rdata = inst5;
            4'd6:  rdata = inst6;
            4'd7:  rdata = inst7;
            4'd8:  rdata = inst8;
            4'd9:  rdata = inst9;
            4'd10: rdata = inst10;
            4'd11: rdata = inst11;
            4'd12: rdata = inst12;
            4'd13: rdata = inst13;
            4'd14: rdata = inst14;
            4'd15: rdata = inst15;
            default: rdata = 16'hC000; // Standardvärde: HALT instruktion
        endcase
    end

endmodule




