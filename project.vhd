
library IEEE;
use IEEE.std_logic_1164.all;
use IEEE.numeric_std.all;

package Datatype is
    attribute Cpu_Name: string;
    type bool_to_bit_array is array(boolean) of bit;
    constant bool_to_bit: bool_to_bit_array;
    subtype bit_4 is std_ulogic_vector(3 downto 0);
    subtype bit_8 is std_ulogic_vector(7 downto 0);
    subtype bit_16 is std_ulogic_vector(15 downto 0);
    subtype bit_32 is std_ulogic_vector(31 downto 0);
    subtype bus_bit_8 is std_logic_vector(7 downto 0);
    type bit_8_vector is array (natural range <>) of bit_8;
    type alu_operation is (pass, land, lor, lnot, addition, substraction, increment, decrement);
    type shifter_operation is (pass, left, right, rotate);
    function to_integer(A: in bit_8; sign: boolean) return integer;
    function to_bit_8(A: in integer) return bit_8;
    function "+"(A, B: in bit_8) return bit_8;
    function "-"(A, B: in bit_8) return bit_8;
    --function "-"(A, B: in bit_8) return bit_8;
    --function "*"(A, B: in bit_8) return bit_8;
    --function "/"(A, B: in bit_8) return bit_8;
    procedure write_ram(signal address: out std_logic_vector; signal data: out std_logic_vector bus;
        constant addr: in std_ulogic_vector; constant word: in std_logic_vector;
        signal ram_ce, ram_we: out std_logic; signal ready: in std_logic; sync: in boolean := true);
    procedure read_ram(signal address: out std_logic_vector; signal data: in std_logic_vector bus;
        constant addr: in std_ulogic_vector; variable result: out std_logic_vector;
        signal ram_ce, ram_we: out std_logic; signal ready: in std_logic; sync: in boolean := true);
    function to_std_logic(cvt: bit) return std_logic;
end Datatype;

package body Datatype is
    constant bool_to_bit: bool_to_bit_array := (false => '0', true => '1');
    function to_integer(A: in bit_8; sign: boolean) return integer is
        type convert_table is array (std_ulogic) of integer;
        constant convert: convert_table := ('1' => 1, others => 0);
        variable high : natural := A'high;
        variable result: integer;
    begin
        if (sign) then
            high := (high - 1);
        end if;
        for i in high downto A'low loop
            result := (result * 2) + (convert(A(i)));
        end loop;
        return (result);
    end to_integer;

    function to_bit_8(A: in integer) return bit_8 is
    begin
        return (x"00");
    end to_bit_8;

    function "+"(A, B: in bit_8) return bit_8 is
    begin
        return bit_8(to_signed(to_integer(signed(A)) + to_integer(signed(B)), 8));
    end "+";

    function "-"(A, B: in bit_8) return bit_8 is
    begin
        return bit_8(to_signed(to_integer(signed(A)) - to_integer(signed(B)), 8));
    end "-";

    procedure write_ram(signal address: out std_logic_vector; signal data: out std_logic_vector bus;
        constant addr: in std_ulogic_vector; constant word: in std_logic_vector;
        signal ram_ce, ram_we: out std_logic; signal ready: in std_logic; sync: in boolean := true) is
    begin
        address <= std_logic_vector(addr);
        data <= word;
        ram_we <= '1';
        ram_ce <= '1';
        wait until (ready = '1');
        ram_ce <= '0';
        if (sync) then wait until (ready = '0');
        end if;
    end write_ram;

    procedure read_ram(signal address: out std_logic_vector; signal data: in std_logic_vector bus;
        constant addr: in std_ulogic_vector; variable result: out std_logic_vector;
        signal ram_ce, ram_we: out std_logic; signal ready: in std_logic; sync: in boolean := true) is
    begin
        address <= std_logic_vector(addr);
        ram_we <= '0';
        ram_ce <= '1';
        wait until (ready = '1');
        result := data;
        ram_ce <= '0';
        if (sync) then wait until (ready = '0');
        end if;
    end read_ram;

    function to_std_logic(cvt: bit) return std_logic is
    begin
        case (cvt) is
            when '0' => return '0';
            when '1' => return '1';
        end case;
    end to_std_logic;
end Datatype;

library IEEE;
use IEEE.std_logic_1164.all;
use IEEE.numeric_std.all;
use Work.Datatype.all;

entity Mux is
    generic (constant selector: positive);
    port (lines: in bit_8_vector(0 to ((2 ** selector) - 1));
        sel: in std_logic_vector(0 to (selector - 1));
        Q: out bit_8);
end Mux;

architecture RTL of Mux is
begin
    Q <= lines(to_integer(unsigned(sel)));
end RTL;

library IEEE;
use IEEE.std_logic_1164.all;
use Work.Datatype.all;

entity Accumulator is
    port (D: in bit_8;
        write: in std_logic;
        reset: in std_logic;
        Q: out bit_8);
end entity;

architecture RTL of Accumulator is
begin
    process (D, write, reset) is
        variable latch_1, latch_2: bit_8;
    begin
        if (reset = '1') then
            latch_1 := (others => '0');
            latch_2 := (others => '0');
        elsif (write = '1') then latch_1 := D;
        else latch_2 := latch_1;
        end if;
        Q <= latch_2;
    end process;
end RTL;

library IEEE;
use IEEE.std_logic_1164.all;
use IEEE.numeric_std.all;
use Work.Datatype.all;

entity Register_File is
    generic (constant selector: natural);
    port (D: in bit_8;
        write: in std_logic;
        sel: in std_logic_vector(0 to (selector - 1));
        Q: out bit_8);
end entity;

architecture RTL of Register_File is
begin
    process (D, write, sel) is
        variable banks: bit_8_vector(0 to ((2 ** selector) - 1)) := (others => (others => '0'));
    begin
        if (write = '1') then banks(to_integer(unsigned(sel))) := D;
        end if;
        Q <= banks(to_integer(unsigned(sel)));
    end process;
end RTL;

library IEEE;
use IEEE.std_logic_1164.all;
use IEEE.numeric_std.all;
use Work.Datatype.all;

entity ALU is
    port (A, B: in bit_8;
        op: in alu_operation;
        C: out bit_8);
end entity;

architecture RTL of ALU is
begin
    process (A, B, op) is
    begin
        case (op) is
            when pass => C <= A;
            when land => C <= (A and B);
            when lor => C <= (A or B);
            when lnot => C <= (not A);
            when addition => C <= (A + B);
            when substraction => C <= (A - B);
            when increment => C <= (A + x"01");
            when decrement => C <= (A - x"01");
        end case;
    end process;
end RTL;

library IEEE;
use IEEE.std_logic_1164.all;
use Work.Datatype.All;

entity Shifter is
    port (D: in bit_8;
        op: in shifter_operation;
        Q: out bit_8);
end Shifter;

architecture RTL of Shifter is
begin
    process (D, op) is
    begin
        case (op) is
            when pass => Q <= D;
            when left => Q <= (D(6 downto 0) & '0');
            when right => Q <= ('0' & D(7 downto 1));
            when rotate => Q <= (D(0) & D(7 downto 1));
        end case;
    end process;
end RTL;

library IEEE;
use IEEE.std_logic_1164.all;
use Work.Datatype.all;

entity Buff is
    port (A: in bit_8;
        OE: in std_logic;
        B: out bus_bit_8 bus);
end Buff;

architecture RTL of Buff is
    signal guard: boolean;
begin
    guard <= (OE = '1');
    B <= guarded bus_bit_8(A);
end RTL;

library IEEE;
use IEEE.std_logic_1164.all;
use Work.Datatype.all;

entity Datapath is
    port (input_dp: in bit_8;
        imm_dp: in bit_8;
        muxsel_dp: in std_logic_vector(0 to 1);
        zero_dp: out std_logic;
        positive_dp: out std_logic;
        accwr_dp: in std_logic;
        rst_dp: in std_logic;
        rfwr_dp: in std_logic;
        rfaddr_dp: in std_logic_vector(0 to 2);
        alusel_dp: in alu_operation;
        shifter_dp: in shifter_operation;
        outen_dp: in std_logic;
        output_dp: out bus_bit_8 bus);
end Datapath;

architecture RTL of Datapath is
    type convert_table is array(boolean) of std_logic;
    constant convert: convert_table := (false => '0', true => '1');
    signal C_rfout, C_shiftout: bit_8;
    signal C_muxout: bit_8;
    signal C_account: bit_8;
    signal C_aluout: bit_8;
    signal aggregate: bit_8_vector(0 to 3);
begin
    aggregate <= (C_shiftout, C_rfout, input_dp, imm_dp);
    MUX: entity Work.Mux generic map (2) port map (aggregate, muxsel_dp, C_muxout);
    zero_dp <= convert((C_muxout nor x"00") = x"FF");
    positive_dp <= (not C_muxout(7));
    ACC: entity Work.Accumulator port map (C_muxout, accwr_dp, rst_dp, C_account);
    REG: entity Work.Register_File generic map (3) port map (C_account, rfwr_dp, rfaddr_dp, C_rfout);
    ALU: entity Work.ALU port map (C_account, C_rfout, alusel_dp, C_aluout);
    SHT: entity Work.Shifter port map (C_aluout, shifter_dp, C_shiftout);
    BUF: entity Work.Buff port map (C_account, outen_dp, output_dp);
end RTL;

library IEEE;
use IEEE.std_logic_1164.all;
use IEEE.numeric_std.all;
use Std.TextIO.all;

entity Memory is
    generic (constant address_width, data_width: positive);
    port (Clk: in std_logic;
        D: inout std_logic_vector((data_width - 1) downto 0) bus;
        A: in std_ulogic_vector((address_width - 1) downto 0);
        CE: in std_logic;
        WE: in std_logic;
        RO: in std_logic;
        Reset: in std_logic;
        Ready: out std_logic;
        Path_Dump: in string;
        Dump: in std_logic;
        Path_Load: in string;
        Load: in std_logic);
end Memory;

architecture RTL of Memory is
begin
    process (Clk, Reset, Dump, Load) is
        subtype memory_range is natural range 0 to ((2 ** address_width) - 1);
        type memory_array is array (memory_range) of std_logic_vector((data_width - 1) downto 0);
        variable memory_cell: memory_array;
        variable index: memory_range := 0;
        file target: text;
        variable text_line: line;
        procedure dump_to_file is
        begin
            write(output, "DUMP MEMORY INTO FILE: " & path_dump & LF);
            file_open(target, path_dump, write_mode);
            for i in memory_cell'range loop 
                write(text_line, to_integer(unsigned(memory_cell(i))));
                writeline(target, text_line);
            end loop;
            file_close(target);
        end dump_to_file;
        procedure load_from_file is
            variable duplicate: line;
            variable need_free: boolean;
        begin
            write(output, "LOAD MEMORY FROM FILE: " & path_load & LF);
            file_open(target, path_load, read_mode);
            for i in memory_cell'range loop
                exit when endfile(target);
                duplicate := null;
                need_free := false;
                readline(target, text_line);
                for j in text_line.all'range loop
                    if ((text_line.all(j) = ' ') and (j /= text_line.all'low)) then
                        duplicate := new string(text_line.all'low to j);
                        duplicate.all := text_line.all(text_line.all'low to j);
                        need_free := true;
                        exit;
                    end if;
                end loop;
                if (duplicate = null) then
                    duplicate := text_line;
                end if;
                memory_cell(i) := std_logic_vector(to_unsigned(integer'value(duplicate.all), data_width));
                if (need_free) then
                    deallocate(duplicate);
                end if;
            end loop;
            file_close(target);
        end load_from_file;
    begin
        text_line := null;
        if (rising_edge(Dump)) then dump_to_file;
        elsif (rising_edge(Load)) then load_from_file;
        end if;
        D <= null;
        Ready <= '0';
        if (Reset = '1') then
            memory_cell := (others => (others => '0'));
        elsif (rising_edge(Clk) and (CE = '1')) then
            index := memory_range(to_integer(unsigned(A)));
            if (WE = '1') then 
                if (RO = '0') then memory_cell(index) := D;
                end if;
            else D <= memory_cell(index);
            end if;
            Ready <= '1';
        end if;
    end process;
end RTL;

library IEEE;
use IEEE.std_logic_1164.all;
use IEEE.numeric_std.all;
use Work.Datatype.all;

entity Fifo is
    generic (constant depth: positive range 1 to 16;
        constant data_width: positive);
    port (Clk: in std_logic;
        D: in std_logic_vector((data_width - 1) downto 0);
        Q: out std_logic_vector((data_width - 1) downto 0);
        Push: in std_logic;
        Pop: in std_logic;
        Empty: out std_logic;
        Full: out std_logic;
        Length: out bit_4;
        Flush: in std_logic);
end Fifo;

architecture RTL of Fifo is
begin
    process (Clk, Flush) is
        subtype index_range is natural range 0 to (depth - 1);
        function inc(index: index_range) return index_range is
        begin
            if (index = index_range'high) then return (index_range'low);
            else return (index + 1);
            end if;
        end inc;
        type fifo_stack_array is array (index_range) of std_logic_vector((data_width - 1) downto 0);
        variable fifo_stack: fifo_stack_array;
        variable count: natural;
        variable first, last: index_range := 0;
        variable empty_logic, full_logic: std_logic;
        variable lenstr: std_logic_vector(4 downto 0);
    begin
        if (Flush = '1') then
            fifo_stack := (others => (others => '0'));
            count := 0;
            first := 0;
            last := 0;
        elsif (rising_edge(Clk)) then
            if ((Push = '1') and (count /= depth)) then
                fifo_stack(last) := D;
                count := (count + 1);
                last := inc(last);
            end if;
            if ((Pop = '1') and (count /= 0)) then
                Q <= fifo_stack(first);
                count := (count - 1);
                first := inc(first);
            end if;
        end if;
        lenstr := std_logic_vector(to_unsigned(count, 5));
        Empty <= to_std_logic(bool_to_bit(count = 0));
        Full <= lenstr(4);
        Length <= bit_4(lenstr(3 downto 0));
    end process;
end RTL;

library IEEE;
use IEEE.std_logic_1164.all;
use IEEE.numeric_std.all;
use Std.TEXTIO.all;
use Work.Datatype.all;

entity Control_Unit is
    port (rst_cpu: in std_logic;
        phi1: in std_logic;
        phi2: in std_logic;
        ready: in std_logic;
        address: out bit_8 := x"00";
        data: inout bus_bit_8 bus;
        ram_ce: out std_logic := '0';
        ram_we: out std_logic := '0';
        ram_ro: out std_logic := '0';
        wake_up: in std_logic;
        input_dp: out bit_8;
        imm_dp: out bit_8;
        muxsel_dp: out std_logic_vector(0 to 1) := "00";
        zero_dp: in std_logic;
        positive_dp: in std_logic;
        accwr_dp: out std_logic;
        rst_dp: out std_logic;
        rfwr_dp: out std_logic;
        rfaddr_dp: out std_logic_vector(0 to 2) := "000";
        alusel_dp: out alu_operation;
        shifter_dp: out shifter_operation;
        outen_dp: out std_logic;
        output_dp: in bus_bit_8 bus);
end Control_Unit;

architecture RTL of Control_Unit is
    constant NOP: bit_4 := "0000";
    constant LDA: bit_4 := "0001";
    constant STA: bit_4 := "0010";
    constant LDM: bit_4 := "0011";
    constant STM: bit_4 := "0100";
    constant LDI: bit_4 := "0101";
    constant JMP: bit_4 := "0110";
    constant JZ: bit_4 := "0111";
    constant JNZ: bit_4 := "1000";
    constant JP: bit_4 := "1001";
    constant LAND: bit_4 := "1010";
    constant LOR: bit_4 := "1011";
    constant ADD: bit_4 := "1100";
    constant SUB: bit_4 := "1101";
    constant SNGL: bit_4 := "1110";
    constant MISC: bit_4 := "1111";
    constant LNOT: bit_4 := "0000";
    constant INC: bit_4 := "0001";
    constant DEC: bit_4 := "0010";
    constant SHFL: bit_4 := "0011";
    constant SHFR: bit_4 := "0100";
    constant ROTR: bit_4 := "0101";
    constant INP: bit_4 := "0000";
    constant OUTP: bit_4 := "0001";
    constant HALT: bit_4 := "0010";
    constant HighZ: bus_bit_8 := "ZZZZZZZZ";
    type State_Logic is (Fetch, Decode_1, Decode_2, Jump_Relative, LDM_Move, STM_Move);
    signal state: State_Logic;
    signal queue_push_in, queue_push_out: bus_bit_8;
    signal queue_push: std_logic;
    signal queue_pop: std_logic;
    signal queue_flush: std_logic;
    signal queue_full, queue_empty: std_logic;
    signal queue_length: bit_4;
    signal PC, PC_fetch: bit_8;
    signal prefetch: boolean;
    signal ram_ce_shared: std_logic;
    signal address_shared: bus_bit_8;
begin
    address <= bit_8(address_shared) when (address_shared /= HighZ) else unaffected;
    ram_ce <= '0' when (ram_ce_shared = 'L') else '1'; -- PULL DOWN
    F0: entity Work.Fifo generic map (16, 8) port map (phi2, queue_push_in, queue_push_out, queue_push,
        queue_pop, queue_empty, queue_full, queue_length, queue_flush);
    process is
        variable byte_read: bus_bit_8;
        variable absolute: bit_8;
        variable displacement: natural := 0;
        variable first: boolean;
    begin
        if ((rst_cpu = '1') or (queue_flush = '1')) then
            PC_fetch <= x"00";
            if (rst_cpu = '1') then
                first := true;
            end if;
        elsif (rising_edge(phi1) and (queue_full = '0') and prefetch) then
            if (queue_flush = '1') then
                displacement := to_integer(unsigned(PC));
            elsif (not first) then
                displacement := (displacement + 1);
                if (displacement > 16#FF#) then
                    displacement := 0;
                end if;
            end if;
            absolute := bit_8(to_unsigned(displacement, 8));
            PC_fetch <= absolute;
            queue_push <= to_std_logic(bool_to_bit((queue_flush = '0') and (not first)));
            read_ram(address_shared, data, absolute, byte_read, ram_ce_shared, ram_we, ready, false);
            queue_push_in <= byte_read;
            first := false;
        end if;
        address_shared <= HighZ;
        ram_ce_shared <= 'L';
        ram_we <= 'W';
        queue_push <= '0';
        wait on phi1;
    end process;
    process is
        variable INST1, INST2: bit_8;
        alias OPCODE: bit_4 is INST1(7 downto 4);
        alias OP1: bit_4 is INST1(3 downto 0);
        alias OP2: bit_8 is INST2;
        variable acclt: bus_bit_8;
        variable inlt: integer;
        variable debug: line;
        variable invalid: boolean;
        procedure reset_state is
        begin
            imm_dp <= x"00";
            muxsel_dp <= "00";
            accwr_dp <= '0';
            rfwr_dp <= '0';
            rst_dp <= '0';
            rfaddr_dp <= "000";
            alusel_dp <= pass;
            shifter_dp <= pass;
            outen_dp <= '0';
        end reset_state;
        procedure finish_state is
        begin
            wait until (phi2 = '1');
            state <= Fetch;
        end finish_state;
        procedure fetch_code(first: boolean := true) is
        begin
            queue_pop <= '1';
            wait until (phi2'delayed = '1');
            queue_pop <= '0';
            if (first) then INST1 := bit_8(queue_push_out);
            else INST2 := bit_8(queue_push_out);
            end if;
            PC <= bit_8(to_unsigned(to_integer(unsigned(PC) + 1), 8));
        end fetch_code;
    begin
        data <= null;
        address_shared <= HighZ;
        queue_pop <= '0';
        queue_flush <= '0';
        prefetch <= true;
        invalid := false;
        if (rst_cpu = '1') then
            PC <= x"00";
            INST1 := x"00";
            INST2 := x"00";
            reset_state;
            rst_dp <= '1';
            ram_ro <= '1';
            prefetch <= false;
            queue_flush <= '1';
            state <= Fetch;
        elsif (rising_edge(phi1) and (queue_empty = '0')) then
            case (state) is
                when Fetch =>
                    reset_state;
                    fetch_code;
                    state <= Decode_1;
                when Decode_1 =>
                    case (OPCODE) is
                        when NOP => state <= Fetch;
                        when LDA =>
                            muxsel_dp <= "01";
                            accwr_dp <= '1';
                            rfaddr_dp <= std_logic_vector(OP1(2 downto 0));
                            finish_state;
                        when STA =>
                            rfwr_dp <= '1';
                            rfaddr_dp <= std_logic_vector(OP1(2 downto 0));
                            finish_state;
                        when LDM | STM | LDI | JMP | JNZ | JZ | JP =>
                            if (((OPCODE = JMP) or (OPCODE = JNZ) or (OPCODE = JZ) or (OPCODE = JP)) and (OP1 /= "0000")) then
                                state <= Jump_Relative;
                            else
                                fetch_code(false);
                                state <= Decode_2;
                                if (OPCODE = LDM) then
                                    prefetch <= false;
                                end if;
                            end if;
                        when LAND | LOR | ADD | SUB | SNGL =>
                            rfaddr_dp <= std_logic_vector(OP1(2 downto 0));
                            case (OPCODE) is
                                when LAND => alusel_dp <= Work.Datatype.land;
                                when LOR => alusel_dp <= Work.Datatype.lor;
                                when ADD => alusel_dp <= addition;
                                when SUB => alusel_dp <= substraction;
                                when SNGL =>
                                    rfaddr_dp <= "000";
                                    case (OP1) is
                                        when LNOT => alusel_dp <= Work.Datatype.lnot;
                                        when INC => alusel_dp <= increment;
                                        when DEC => alusel_dp <= decrement;
                                        when SHFL => shifter_dp <= left;
                                        when SHFR => shifter_dp <= right;
                                        when ROTR => shifter_dp <= rotate;
                                        when others => invalid := true;
                                    end case;
                                when others => null;
                            end case;
                            wait until (phi2 = '1');
                            accwr_dp <= '1';
                            state <= Fetch;
                        when MISC =>
                            case (OP1) is
                                when INP =>
                                    readline(input, debug);
                                    read(debug, inlt);
                                    muxsel_dp <= "10";
                                    input_dp <= bit_8(to_unsigned(inlt, 8));
                                    accwr_dp <= '1';
                                    finish_state;
                                when OUTP =>
                                    outen_dp <= '1';
                                    wait until (phi2 = '1');
                                    write(output, integer'image(to_integer(unsigned(output_dp))) & LF);
                                    state <= Fetch;
                                when HALT =>
                                    write(output, "CPU HALTED" & LF);
                                    prefetch <= false;
                                    wait until (wake_up = '1');
                                    finish_state;
                                when others => invalid := true;
                            end case;
                        when others => invalid := true;
                    end case;
                when Decode_2 =>
                    case (OPCODE) is
                        when LDM =>
                            read_ram(address_shared, data, OP2, acclt, ram_ce_shared, ram_we, ready, false);
                            state <= LDM_Move;
                        when STM =>
                            outen_dp <= '1';
                            wait until (phi2 = '1');
                            outen_dp <= '0';
                            acclt := output_dp;
                            prefetch <= false;
                            state <= STM_Move;
                        when LDI =>
                            muxsel_dp <= "11";
                            imm_dp <= bit_8(OP2);
                            accwr_dp <= '1';
                            finish_state;
                        when JMP | JZ | JNZ =>
                            if ((OPCODE = JMP) or ((OPCODE = JZ) and (zero_dp = '1')) or
                                    ((OPCODE = JNZ) and (zero_dp = '0')) or ((OPCODE = JP) and (positive_dp = '1'))) then
                                PC <= OP2;
                                queue_flush <= '1';
                            end if;
                            finish_state;
                        when others => null;
                    end case;
                when Jump_Relative => null;
                when LDM_Move =>
                    muxsel_dp <= "10";
                    input_dp <= bit_8(acclt);
                    accwr_dp <= '1';
                    finish_state;
                when STM_Move =>
                    write_ram(address_shared, data, OP2, acclt, ram_ce_shared, ram_we, ready, false);
                    state <= Fetch;
            end case;
        end if;
        if (invalid) then
            report "INVALID INSTRUCTION" severity warning;
            state <= Fetch;
        end if;
        ram_ce_shared <= 'L';
        ram_we <= 'W';
        wait on phi1, rst_cpu;
    end process;
end RTL;

library IEEE;
use IEEE.std_logic_1164.all;
use Work.Datatype.all;

entity CPU is
    port (rst_cpu: in std_logic;
        phi1: in std_logic;
        phi2: in std_logic;
        ready: in std_logic;
        address: out bit_8;
        data: inout bus_bit_8 bus;
        ram_ce: out std_logic;
        ram_we: out std_logic;
        ram_ro: out std_logic := '0';
        wake_up: in std_logic);
    attribute Cpu_Name of CPU: entity is "MaoKo CPU";
end CPU;

architecture RTL of CPU is
    signal input_dp: bit_8;
    signal imm_dp: bit_8;
    signal muxsel_dp: std_logic_vector(0 to 1);
    signal zero_dp: std_logic;
    signal positive_dp: std_logic;
    signal accwr_dp: std_logic;
    signal rst_dp: std_logic;
    signal rfwr_dp: std_logic;
    signal rfaddr_dp: std_logic_vector(0 to 2);
    signal alusel_dp: alu_operation;
    signal shifter_dp: shifter_operation;
    signal outen_dp: std_logic;
    signal output_dp: bus_bit_8 bus;
begin
    C0: entity Work.Control_Unit port map (rst_cpu, phi1, phi2, ready, address, data, ram_ce, ram_we, ram_ro, wake_up,
        input_dp, imm_dp, muxsel_dp, zero_dp, positive_dp, accwr_dp, rst_dp, rfwr_dp, rfaddr_dp, alusel_dp,
        shifter_dp, outen_dp, output_dp);
    D0: entity Work.Datapath port map (input_dp, imm_dp, muxsel_dp, zero_dp, positive_dp, accwr_dp, rst_dp, rfwr_dp,
        rfaddr_dp, alusel_dp, shifter_dp, outen_dp, output_dp);
end RTL;

library IEEE;
use IEEE.std_logic_1164.all;

entity Clock is
    generic (constant period: time);
    port (clk: inout std_logic := '1';
        clki: out std_logic := '0';
        clk2: inout std_logic := '0');
end Clock;

architecture RTL of Clock is
begin
    process is
    begin
        clk <= '0' after (period / 2);
        clk <= transport '1' after period;
        wait until rising_edge(clk);
    end process;
    clki <= (not clk);
    clk2 <= (not clk2) when rising_edge(clk);
end RTL;

library IEEE;
use IEEE.std_logic_1164.all;
use Std.TextIO.all;
use Work.Datatype.all;

entity Test is
end Test;

architecture Bench of Test is
    signal reset: std_logic;
    signal ready: std_logic;
    signal address: bit_8 := x"00";
    signal data: bus_bit_8 bus;
    signal ram_ce:  std_logic;
    signal ram_we: std_logic;
    signal ram_ro: std_logic := '0';
    signal dump: std_logic := '0';
    signal load: std_logic := '0';
    signal wake_up: std_logic;
    signal clk: std_logic;
    signal clki: std_logic;
    signal clk2: std_logic;
begin
    --rst_cpu <= '0';
    --clk_cpu <= '1';
    --wake_up <= '0';
    CPU0: entity Work.CPU port map (reset, clk, clki, ready, address, data, ram_ce, ram_we, ram_ro, wake_up);
    RAM0: entity Work.Memory generic map (8, 8) port map (clki, data, address,
        ram_ce, ram_we, ram_ro, reset, ready, "RAM.txt", dump, "CODE.txt", load);
    CLK0: entity Work.Clock generic map (6 fs) port map (clk, clki, clk2);
    process is
        variable result: bus_bit_8;
    begin
        write(output, "** POWER UP " & Work.CPU'Cpu_Name & " **" & LF);
        wake_up <= '0';
        --wait until (clk = '1');
        --write_ram(address, data, x"00", x"FF", ram_ce, ram_we, ready);
        --wait for 5 fs;
        --write_ram(address, data, x"01", x"EE", ram_ce, ram_we, ready);
        --wait for 5 fs;
        --data <= null;
        --read_ram(address, data, x"00", result, ram_ce, ram_we, ready);
        --data <= result;
        --wait for 3 fs;
        --write_ram(address, data, x"02", x"DD", ram_ce, ram_we, ready);
        --write_ram(address, data, x"03", x"CC", ram_ce, ram_we, ready);
        --write_ram(address, data, x"04", x"BB", ram_ce, ram_we, ready);
        --write_ram(address, data, x"05", x"AA", ram_ce, ram_we, ready);
        --write_ram(address, data, x"06", x"99", ram_ce, ram_we, ready);
        --wait for 3 fs;
        --dump <= '1';
        --wait for 3 fs;
        --dump <= '0';
        --wait for 3 fs;
        --load <= '1';
        --wait for 3 fs;
        --load <= '0';
        --data <= null;
        --data <= result;
        --wait for 3 fs;
        wait until (reset = '0');
        load <= '1';
        wait for 3 fs;
        load <= '0';
        wait for 300 fs;
        wake_up <= '1';
        wait for 5 fs;
        wake_up <= '0';
        --read_ram(address, data, x"00", result, ram_ce, ram_we, ready);
        wait;
    end process;
    process is
    begin
        reset <= '1';
        wait until (clki = '1');
        wait until (clki = '1');
        reset <= '0';
        wait;
    end process;
end Bench;

