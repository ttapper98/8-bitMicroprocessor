-- FA
library ieee;
use ieee.std_logic_1164.all;

entity Full_Adder is
   port( X, Y, Cin : in std_logic:='0';
         Sum, Cout : out std_logic:='0');
end Full_Adder;
 
architecture Behav_FA of Full_Adder is
begin
   Sum <= (X xor Y) xor Cin;
   Cout <= (X and (Y or Cin)) or (Cin and Y);
end Behav_FA;


--8-bit Adder/Subtractor
library ieee;
use ieee.std_logic_1164.all;

entity ADD_SUB is
   port( AS: in std_logic :='0';
          A_in,B_in  : in std_logic_vector(7 downto 0):="00000000";
          Z_out  : out std_logic_vector(7 downto 0):="00000000";
          Crry_flg, OV_flg : out std_logic:= '0');
end ADD_SUB;
 
architecture struct_AS of ADD_SUB is
component Full_Adder is
  port( X, Y, Cin : in std_logic := '0';
        sum, Cout : out std_logic:= '0');
end component;
signal C: std_logic_vector(7 downto 0) :="00000000";
signal TMP: std_logic_vector(7 downto 0):="00000000";
 
begin
TMP(0) <= AS xor B_in(0);
TMP(1) <= AS xor B_in(1);
TMP(2) <= AS xor B_in(2);
TMP(3) <= AS xor B_in(3);
TMP(4) <= AS xor B_in(4);
TMP(5) <= AS xor B_in(5);
TMP(6) <= AS xor B_in(6);
TMP(7) <= AS xor B_in(7);

FA0:Full_Adder port map(A_in(0),TMP(0),AS, Z_out(0),C(0));-- R0
FA1:Full_Adder port map(A_in(1),TMP(1),C(0), Z_out(1),C(1));-- R1
FA2:Full_Adder port map(A_in(2),TMP(2),C(1), Z_out(2),C(2));-- R2
FA3:Full_Adder port map(A_in(3),TMP(3),C(2), Z_out(3),C(3));-- R3

FA4:Full_Adder port map(A_in(4),TMP(4),C(3), Z_out(4),C(4));-- R4
FA5:Full_Adder port map(A_in(5),TMP(5),C(4), Z_out(5),C(5));-- R5
FA6:Full_Adder port map(A_in(6),TMP(6),C(5), Z_out(6),C(6));-- R6
FA7:Full_Adder port map(A_in(7),TMP(7),C(6), Z_out(7),C(7));-- R7
OV_flg <= C(6) XOR C(7);
Crry_flg <= C(7);
end struct_AS;

library IEEE;
use IEEE.STD_LOGIC_1164.ALL;


entity register_8_bit is 
port (clk: in std_logic :='0';
		reset: in std_logic :='0';
		En: in std_logic :='0';
		in_vec: in std_logic_vector (7 downto 0) := "00000000";
		out_vec: out std_logic_vector (7 downto 0) := "00000000");
end register_8_bit;
architecture struct1 of register_8_bit is 

begin
process (clk,En,reset)
begin 
	If reset = '1' then 
		Out_vec <= "00000000";
	elsif En = '1' then 
		if rising_edge(clk) then 
		out_vec <= in_vec;
		end if; 
	end if;
end process;
end struct1;


Library IEEE;
USE IEEE.Std_logic_1164.all;

entity DFF_fallingEdge is 
   port(
		Clk :in std_logic :='0';  
		reset: in std_logic :='0';
		D :in  std_logic :='0'; 
      Q : out std_logic :='0';   
		QN : out std_logic :='1');
end DFF_fallingEdge;
architecture Behavioral of DFF_fallingEdge is  
begin  
 process(Clk)
 begin 
	if falling_edge(Clk) then
		if reset ='1' then 
			Q <= '0';
			QN <= '1';
		else 
			Q <= D;
			QN <= not D;
		end if;
   end if;       
 end process;  
end Behavioral; 

Library IEEE;
USE IEEE.Std_logic_1164.all;

entity DFF_risingEdge is 
   port(
		Clk :in std_logic :='0';  
		reset: in std_logic :='0';
		D :in  std_logic :='0'; 
      Q : out std_logic :='0';   
		QN : out std_logic :='1' );
end DFF_risingEdge;
architecture Behavioral2 of DFF_risingEdge is  
begin  
 process(Clk)
 begin 
	if rising_edge(Clk) then
		if reset ='1' then 
			Q <= '0';
			QN <= '1';
		else 
			Q <= D;
			QN <= not D;
		end if;
   end if;       
 end process;  
end Behavioral2; 

library IEEE;
use IEEE.STD_LOGIC_1164.ALL;
USE ieee.numeric_std.ALL;

-- A 128x8 single-port RAM in VHDL
entity Memory_256x8 is
port(
 clk: in std_logic := '0';
 RW_en: in std_logic := '0'; 
 mem_addr: in std_logic_vector(7 downto 0); -- Address to write/read 
 mem_data_in: in std_logic_vector(7 downto 0); -- data that is to be writen to address 
 mem_data_out: out std_logic_vector(7 downto 0)); -- Data out
end Memory_256x8;

architecture Behavioral3 of Memory_256x8 is

type MEM_ARRAY is array (0 to 255 ) of std_logic_vector (7 downto 0);
-- initial values in the RAM
signal MEM: MEM_ARRAY :=(
   x"10",x"F8",x"A0",x"00",-- 0x00: 
   x"10",x"FC",x"50",x"01",-- 0x04: 
   x"40",x"EC",x"60",x"EC",-- 0x08: 
   x"70",x"01",x"80",x"00",-- 0x0C: 
   x"90",x"05",x"00",x"00",-- 0x10: 
   x"00",x"00",x"00",x"00",-- 0x14: 
   x"00",x"00",x"00",x"00",-- 0x18: 
   x"00",x"00",x"00",x"00",-- 0x1C: 
   x"00",x"00",x"00",x"00",-- 0x20: 
   x"00",x"00",x"00",x"00",-- 0x24: 
   x"00",x"00",x"00",x"00",-- 0x28: 
   x"00",x"00",x"00",x"00",-- 0x2C: 
   x"00",x"00",x"00",x"00",-- 0x30: 
   x"00",x"00",x"00",x"00",-- 0x34: 
   x"00",x"00",x"00",x"00",-- 0x38: 
   x"00",x"00",x"00",x"00",-- 0x3C: 
   x"00",x"00",x"00",x"00",-- 0x40: 
   x"00",x"00",x"00",x"00",-- 0x44: 
   x"00",x"00",x"00",x"00",-- 0x48: 
   x"00",x"00",x"00",x"00",-- 0x4C: 
   x"00",x"00",x"00",x"00",-- 0x50: 
   x"00",x"00",x"00",x"00",-- 0x54: 
   x"00",x"00",x"00",x"00",-- 0x58: 
   x"00",x"00",x"00",x"00",-- 0x5C: 
   x"00",x"00",x"00",x"00",-- 0x60:
   x"00",x"00",x"00",x"00",-- 0x64:
   x"00",x"00",x"00",x"00",-- 0x68:
   x"00",x"00",x"00",x"00",-- 0x6C:
   x"00",x"00",x"00",x"00",-- 0x70:
   x"00",x"00",x"00",x"00",-- 0x74:
   x"00",x"00",x"00",x"00",-- 0x78:
   x"00",x"00",x"00",x"00",-- 0x7C:
   x"00",x"00",x"00",x"00",-- 0x80: 
   x"00",x"00",x"00",x"00",-- 0x84: 
   x"00",x"00",x"00",x"00",-- 0x88: 
   x"00",x"00",x"00",x"00",-- 0x8C: 
   x"00",x"00",x"00",x"00",-- 0x90: 
   x"00",x"00",x"00",x"00",-- 0x94: 
   x"00",x"00",x"00",x"00",-- 0x98: 
   x"00",x"00",x"00",x"00",-- 0x9C: 
   x"00",x"00",x"00",x"00",-- 0xA0: 
   x"00",x"00",x"00",x"00",-- 0xA4: 
   x"00",x"00",x"00",x"00",-- 0xA8: 
   x"00",x"00",x"00",x"00",-- 0xAC: 
   x"00",x"00",x"00",x"00",-- 0xB0: 
   x"00",x"00",x"00",x"00",-- 0xB4: 
   x"00",x"00",x"00",x"00",-- 0xB8: 
   x"00",x"00",x"00",x"00",-- 0xBC: 
   x"00",x"00",x"00",x"00",-- 0xC0: 
   x"00",x"00",x"00",x"00",-- 0xC4: 
   x"00",x"00",x"00",x"00",-- 0xC8: 
   x"00",x"00",x"00",x"00",-- 0xCC: 
   x"00",x"00",x"00",x"00",-- 0xD0: 
   x"00",x"00",x"00",x"00",-- 0xD4: 
   x"00",x"00",x"00",x"00",-- 0xD8: 
   x"00",x"00",x"00",x"00",-- 0xDC: 
   x"00",x"00",x"00",x"00",-- 0xE0:
   x"00",x"00",x"00",x"00",-- 0xE4:
   x"00",x"00",x"00",x"00",-- 0xE8:
   x"03",x"00",x"00",x"00",-- 0xEC:
   x"00",x"00",x"00",x"00",-- 0xF0:
   x"00",x"00",x"00",x"00",-- 0xF4:
   x"01",x"00",x"00",x"00",-- 0xF8:
   x"02",x"00",x"00",x"00"	-- 0xFC:
   ); 
begin
process(clk)
begin
 if rising_edge(clk) then
	if RW_en= '1' then -- when write enable = 1, 
		-- write input data into RAM at the provided address
		MEM(to_integer(unsigned(mem_addr))) <= mem_data_in;
		-- The index of the RAM array type needs to be integer so
		-- converts RAM_ADDR from std_logic_vector -> Unsigned -> Interger using numeric_std library
	end if;
 end if;
end process;
 -- Data to be read out 
 mem_data_out <= mem(to_integer(unsigned(mem_addr)));
end Behavioral3;


library IEEE;
use IEEE.STD_LOGIC_1164.ALL;
USE ieee.numeric_std.ALL;


entity ALU_8_bit is 
 port ( A_in, B_in: in std_logic_vector (7 downto 0):="00000000";
			ALU_Ctrl: in std_logic_vector (2 downto 0):="000"; 
			Z_out: out std_logic_vector (7 downto 0):="00000000";
			ZF,CF,VF: out std_logic := '0');
 end ALU_8_bit;
 architecture Struct_ALU of ALU_8_bit is 
 
	component ADD_SUB is 
		port ( AS: in std_logic :='0';
          A_in,B_in  : in std_logic_vector(7 downto 0):="00000000";
          Z_out  : out std_logic_vector(7 downto 0):="00000000";
          Crry_flg, OV_flg : out std_logic:= '0');
	end component;

 signal Added_value: std_logic_vector (7 downto 0) := "00000000";
 signal ACF,AOVF: std_logic := '0';
 signal Subtracted_value: std_logic_vector (7 downto 0) := "00000000";
 signal SCF,SOVF: std_logic := '0';
 
 begin
 
 Xadd: ADD_SUB port map ('0', A_in, B_in, added_Value, ACF, AOVF);
 Xsub: ADD_SUB port map ('1', A_in, B_in, subtracted_Value, SCF, SOVF);
 
 process (A_in, B_in, ALU_Ctrl,added_Value,subtracted_Value,ACF,SCF,AOVF,SOVF)
 begin 
	case ALU_Ctrl is 
	when "000" => -- add A and B
		z_out <= added_Value;
		
		if added_Value = "00000000" then 
			ZF <= '1';
		else ZF <= '0';
		end if;
		CF <= ACF;
		VF <= AOVF;
		
	when "001" => -- sub B from A
		z_out <= subtracted_Value;
		
		if subtracted_Value = "00000000" then 
			ZF <= '1';
		else ZF <= '0';
		end if;
		CF <= SCF;
		VF <= SOVF;
		
	when "010" => -- copy A
		z_out <= A_in;
		
		if A_in = "00000000" then 
			ZF <= '1';
		else ZF <= '0';
		end if;
		CF <= '0';
		VF <= '0';
		
	when "011" => -- copy B
		Z_out <= B_in;
		
		if B_in = "00000000" then 
			ZF <= '1';
		else ZF <= '0';
		end if;
		CF <= '0';
		VF <= '0';
	When others => null;
	
		
	end case; 
 end process;
 
 end Struct_ALU;


library IEEE;
use IEEE.STD_LOGIC_1164.ALL;
use IEEE.NUMERIC_STD.ALL;

entity multiplier is
Port(A: in std_logic_vector (7 downto 0):= x"00";
	  B: in std_logic_vector (7 downto 0) := x"00";
	  CLK: in std_logic := '0';
	  Z: out std_logic_vector (15 downto 0) := x"0000");
end multiplier;

architecture multBehavioral of multiplier is
signal inputA: signed (7 downto 0);
signal inputB: signed (7 downto 0);
signal output: signed (15 downto 0);
begin

inputA <= signed(A);
inputB <= signed(B);

process(inputA, inputB, CLK)
begin
	output <= inputA * inputB; --if issues occur add a delay to output to account for race condition w/ A,B,CLK
	if rising_edge(CLK) then
		Z <= std_logic_vector(output);
	end if;
end process;
end multBehavioral;

library IEEE;
use IEEE.STD_LOGIC_1164.ALL;

entity mux_4 is
Port(a0: in std_logic_vector (7 downto 0) := x"00";
	  a1: in std_logic_vector (7 downto 0) := x"00";
	  a2: in std_logic_vector (7 downto 0) := x"00";
	  a3: in std_logic_vector (7 downto 0) := x"00";
	  sel: in std_logic_vector (1 downto 0):= "00";
	  Z: out std_logic_vector (7 downto 0) := x"00");
end mux_4;

architecture mux4Behavioral of mux_4 is
begin
process(a0,a1,a2,a3,sel)
begin
	case sel is
		when "00" => z <= a0;
		when "01" => z <= a1;
		when "10" => z <= a2;
		when others => z <= a3;
	end case;
end process;
end mux4Behavioral;

library IEEE;
use IEEE.STD_LOGIC_1164.ALL;

entity mux_2 is
Port(a0: in std_logic_vector (7 downto 0):= x"00";
	  a1: in std_logic_vector (7 downto 0):= x"00";
	  sel: in std_logic := '0';
	  Z: out std_logic_vector (7 downto 0):= x"00");
end mux_2;

architecture mux2Behavioral of mux_2 is
begin
process(a0,a1,sel)
begin
	case sel is
		when '0' => z <= a0;
		when others => z <= a1;
	end case;
end process;
end mux2Behavioral;

library IEEE;
use IEEE.STD_LOGIC_1164.ALL;
use IEEE.NUMERIC_STD.ALL;

entity CU is
	Port(Inst: in std_logic_vector (7 downto 0) :="00000000";
			Enable, load_mode: in std_logic :='0';
			CLK: in std_logic :='0';
			PCE, IRE, MDRE, MARE, AE, BE, SRE, PCreset, program: out std_logic :='0';
			MemWR, IorD, Pause, ALUSRCA, PCSRC, JZ, JC, JV: out std_logic :='0';
			ASRC, BSRC, ALUSRCB: out std_logic_vector (1 downto 0):="00";
			ALUCTRL: out std_logic_vector (2 downto 0):="000");
end CU;

architecture CUBehavioral of CU is
signal counter,counter2 : integer := 0;
begin
process(Inst, Enable, CLK, load_mode)
Begin
if load_mode = '1' then
	if rising_edge(CLK) then
		if counter < 255 then
			if counter = 0 then
				PCreset <= '1';
				PCSRC <= '0';
				PCE <= '1';
			
				IRE <= '0';
				MDRE <= '0';
				MARE <= '0';
				AE <= '0'; 
				BE <= '0';
				SRE <= '0';
				MemWR <= '0';
			
				IorD <= '0';
				program <= '1';
				ALUSRCA <= '0';
				ALUSRCB <= "01";
				ALUCTRL <= "000";
				ASRC <= "00";
				BSRC <= "00";

				JZ <= '0';
				JV <= '0';
				JC <= '0';
				Pause <= '0';
			
				counter <= counter + 1;
			else 
				PCreset <= '0';
				PCSRC <= '0';
				PCE <= '1';
			
				IRE <= '0';
				MDRE <= '0';
				MARE <= '0';
				AE <= '0'; 
				BE <= '0';
				SRE <= '0';
				MemWR <= '0';
			
				IorD <= '0';
				program <= '1';
				ALUSRCA <= '0';
				ALUSRCB <= "01";
				ALUCTRL <= "000";
				ASRC <= "00";
				BSRC <= "00";

				JZ <= '0';
				JV <= '0';
				JC <= '0';
				Pause <= '0';
			
				counter <= counter + 1;
			end if;
		else
			counter <= 0;
		end if;
	end if;
elsif load_mode = '0' then
	PCreset <= '0';
	program <= '0';
	If enable = '1' then 
		if rising_edge(CLK) then
			if counter = 0 and counter2 = 0 then
				
				PCSRC <= '0';
				PCE <= '0';
				
				IRE <= '1';
				MDRE <= '0';
				MARE <= '0';
				AE <= '0'; 
				BE <= '0';
				SRE <= '0';
				MemWR <= '0';
				
				IorD <= '0';
				ALUSRCA <= '0';
				ALUSRCB <= "01";
				ALUCTRL <= "000";
				ASRC <= "00";
				BSRC <= "00";

				JZ <= '0';
				JV <= '0';
				JC <= '0';
				Pause <= '0';
				
				counter2 <= counter2 + 1;
				
			elsif counter = 0 and counter2 = 1 then
				
				PCSRC <= '0';
				PCE <= '1';
				
				IRE <= '0';
				MDRE <= '0';
				MARE <= '0';
				AE <= '0'; 
				BE <= '0';
				SRE <= '0';
				MemWR <= '0';
				
				IorD <= '0';
				ALUSRCA <= '0';
				ALUSRCB <= "01";
				ALUCTRL <= "000";
				ASRC <= "00";
				BSRC <= "00";

				JZ <= '0';
				JV <= '0';
				JC <= '0';
				Pause <= '0';
				
				counter2 <= 0;
				counter <= counter + 1;
			else
				case Inst is
					when "00010000" =>
							if counter = 1 then
								PCSRC <= '0';
								PCE <= '0';
								
								IRE <= '0';
								MDRE <= '0';
								MARE <= '1';
								AE <= '0'; 
								BE <= '0';
								SRE <= '0';
								MemWR <= '0';
								
								IorD <= '0';
								ALUSRCA <= '0';
								ALUSRCB <= "01";
								ALUCTRL <= "000";
								ASRC <= "00";
								BSRC <= "00";

								JZ <= '0';
								JV <= '0';
								JC <= '0';
								Pause <= '0';
								
								counter <= counter + 1;
							elsif counter = 2 then -- cycle 3
								PCSRC <= '0';
								PCE <= '0';
								
								IRE <= '0';
								MDRE <= '1';
								MARE <= '0';
								AE <= '0'; 
								BE <= '0';
								SRE <= '0';
								MemWR <= '0';
								
								IorD <= '1';
								ALUSRCA <= '0';
								ALUSRCB <= "01";
								ALUCTRL <= "000";
								ASRC <= "00";
								BSRC <= "00";

								JZ <= '0';
								JV <= '0';
								JC <= '0';
								Pause <= '0';
								
								counter <= counter + 1;
							elsif counter = 3 then -- cylce 4 
								PCSRC <= '0';
								PCE <= '1';
								
								IRE <= '0';
								MDRE <= '0';
								MARE <= '0';
								AE <= '1'; 
								BE <= '0';
								SRE <= '0';
								MemWR <= '0';
								
								IorD <= '0';
								ALUSRCA <= '0';
								ALUSRCB <= "01";
								ALUCTRL <= "000";
								ASRC <= "01";
								BSRC <= "00";

								JZ <= '0';
								JV <= '0';
								JC <= '0';
								Pause <= '0';
								
								counter <= 0;
							end if;
				when "00100000" => -- load intermediate 
							if counter = 1 then -- cycle 2
								PCSRC <= '0';
								PCE <= '0';
								
								IRE <= '0';
								MDRE <= '1';
								MARE <= '0';
								AE <= '0'; 
								BE <= '0';
								SRE <= '0';
								MemWR <= '0';
								
								IorD <= '0';
								ALUSRCA <= '0';
								ALUSRCB <= "01";
								ALUCTRL <= "000";
								ASRC <= "00";
								BSRC <= "00";

								JZ <= '0';
								JV <= '0';
								JC <= '0';
								Pause <= '0';
								
								counter <= counter + 1;
							elsif counter = 2 then -- cycle 3 
								PCSRC <= '0';
								PCE <= '1';
								
								IRE <= '0';
								MDRE <= '0';
								MARE <= '0';
								AE <= '1'; 
								BE <= '0';
								SRE <= '0';
								MemWR <= '0';
								
								IorD <= '0';
								ALUSRCA <= '0';
								ALUSRCB <= "01";
								ALUCTRL <= "000"; -- add
								ASRC <= "01";
								BSRC <= "00";

								JZ <= '0';
								JV <= '0';
								JC <= '0';
								Pause <= '0';
								
								counter <= 0;
							end if;
				when "00110000" => --STA X
							if counter = 1 then -- cycle 2
								PCSRC <= '0';
								PCE <= '0';
								
								IRE <= '0';
								MDRE <= '0';
								MARE <= '1';
								AE <= '0'; 
								BE <= '0';
								SRE <= '0';
								MemWR <= '0';
								
								IorD <= '0';
								ALUSRCA <= '0';
								ALUSRCB <= "01";
								ALUCTRL <= "000";
								ASRC <= "00";
								BSRC <= "00";

								JZ <= '0';
								JV <= '0';
								JC <= '0';
								Pause <= '0';
								
								counter <= counter + 1;
							elsif counter = 2 then
								PCSRC <= '0';
								PCE <= '1';
								
								IRE <= '0';
								MDRE <= '0';
								MARE <= '0';
								AE <= '0'; 
								BE <= '0';
								SRE <= '0';
								MemWR <= '1';
								
								IorD <= '1';
								ALUSRCA <= '0';
								ALUSRCB <= "01";
								ALUCTRL <= "000";
								ASRC <= "00";
								BSRC <= "00";

								JZ <= '0';
								JV <= '0';
								JC <= '0';
								Pause <= '0';
								
								counter <= 0;
							end if;
				when "01000000" => --ADD X
							if counter = 1 then -- cycle 2
								PCSRC <= '0';
								PCE <= '0';
								
								IRE <= '0';
								MDRE <= '0';
								MARE <= '1';
								AE <= '0'; 
								BE <= '0';
								SRE <= '0';
								MemWR <= '0';
								
								IorD <= '0';
								ALUSRCA <= '0';
								ALUSRCB <= "01";
								ALUCTRL <= "000";
								ASRC <= "00";
								BSRC <= "00";

								JZ <= '0';
								JV <= '0';
								JC <= '0';
								Pause <= '0';
								
								counter <= counter + 1;
							elsif counter = 2 then
								PCSRC <= '0';
								PCE <= '0';
								
								IRE <= '0';
								MDRE <= '1';
								MARE <= '0';
								AE <= '0'; 
								BE <= '0';
								SRE <= '0';
								MemWR <= '0';
								
								IorD <= '1';
								ALUSRCA <= '0';
								ALUSRCB <= "01";
								ALUCTRL <= "000";
								ASRC <= "00";
								BSRC <= "00";

								JZ <= '0';
								JV <= '0';
								JC <= '0';
								Pause <= '0';
								
								counter <= counter + 1;
							elsif counter = 3 then
								PCSRC <= '0';
								PCE <= '0';
								
								IRE <= '0';
								MDRE <= '0';
								MARE <= '0';
								AE <= '0'; 
								BE <= '0';
								SRE <= '1';
								MemWR <= '0';
								
								IorD <= '1';
								ALUSRCA <= '1';
								ALUSRCB <= "10";
								ALUCTRL <= "000";
								ASRC <= "00";
								BSRC <= "00";

								JZ <= '0';
								JV <= '0';
								JC <= '0';
								Pause <= '0';
								
								counter <= counter + 1;
							elsif counter = 4 then
								PCSRC <= '0';
								PCE <= '1';
								
								IRE <= '0';
								MDRE <= '0';
								MARE <= '0';
								AE <= '1'; 
								BE <= '0';
								SRE <= '0';
								MemWR <= '0';
								
								IorD <= '0';
								ALUSRCA <= '0';
								ALUSRCB <= "01";
								ALUCTRL <= "000";
								ASRC <= "00";
								BSRC <= "00";

								JZ <= '0';
								JV <= '0';
								JC <= '0';
								Pause <= '0';
								
								counter <= 0;
							end if;
				when "01010000" => --ADD #X
							if counter = 1 then -- cycle 2
								PCSRC <= '0';
								PCE <= '0';
								
								IRE <= '0';
								MDRE <= '1';
								MARE <= '0';
								AE <= '0'; 
								BE <= '0';
								SRE <= '0';
								MemWR <= '0';
								
								IorD <= '0';
								ALUSRCA <= '0';
								ALUSRCB <= "01";
								ALUCTRL <= "000";
								ASRC <= "00";
								BSRC <= "00";

								JZ <= '0';
								JV <= '0';
								JC <= '0';
								Pause <= '0';
								
								counter <= counter + 1;
							elsif counter = 2 then
								PCSRC <= '0';
								PCE <= '0';
								
								IRE <= '0';
								MDRE <= '0';
								MARE <= '0';
								AE <= '0'; 
								BE <= '0';
								SRE <= '1';
								MemWR <= '0';
								
								IorD <= '0';
								ALUSRCA <= '1';
								ALUSRCB <= "10";
								ALUCTRL <= "000";
								ASRC <= "00";
								BSRC <= "00";

								JZ <= '0';
								JV <= '0';
								JC <= '0';
								Pause <= '0';
								
								counter <= counter + 1;
							elsif counter = 3 then
								PCSRC <= '0';
								PCE <= '1';
								
								IRE <= '0';
								MDRE <= '0';
								MARE <= '0';
								AE <= '1'; 
								BE <= '0';
								SRE <= '0';
								MemWR <= '0';
								
								IorD <= '0';
								ALUSRCA <= '0';
								ALUSRCB <= "01";
								ALUCTRL <= "000";
								ASRC <= "00";
								BSRC <= "00";

								JZ <= '0';
								JV <= '0';
								JC <= '0';
								Pause <= '0';
								
								counter <= 0;
							end if;
				when "01100000" => --SUB X
							if counter = 1 then -- cycle 2
								PCSRC <= '0';
								PCE <= '0';
								
								IRE <= '0';
								MDRE <= '0';
								MARE <= '1';
								AE <= '0'; 
								BE <= '0';
								SRE <= '0';
								MemWR <= '0';
								
								IorD <= '0';
								ALUSRCA <= '0';
								ALUSRCB <= "01";
								ALUCTRL <= "000";
								ASRC <= "00";
								BSRC <= "00";

								JZ <= '0';
								JV <= '0';
								JC <= '0';
								Pause <= '0';
								
								counter <= counter + 1;
							elsif counter = 2 then
								PCSRC <= '0';
								PCE <= '0';
								
								IRE <= '0';
								MDRE <= '1';
								MARE <= '0';
								AE <= '0'; 
								BE <= '0';
								SRE <= '0';
								MemWR <= '0';
								
								IorD <= '1';
								ALUSRCA <= '0';
								ALUSRCB <= "01";
								ALUCTRL <= "000";
								ASRC <= "00";
								BSRC <= "00";

								JZ <= '0';
								JV <= '0';
								JC <= '0';
								Pause <= '0';
								
								counter <= counter + 1;
							elsif counter = 3 then
								PCSRC <= '0';
								PCE <= '0';
								
								IRE <= '0';
								MDRE <= '0';
								MARE <= '0';
								AE <= '0'; 
								BE <= '0';
								SRE <= '1';
								MemWR <= '0';
								
								IorD <= '1';
								ALUSRCA <= '1';
								ALUSRCB <= "10";
								ALUCTRL <= "001";
								ASRC <= "00";
								BSRC <= "00";

								JZ <= '0';
								JV <= '0';
								JC <= '0';
								Pause <= '0';
								
								counter <= counter + 1;
							elsif counter = 4 then
								PCSRC <= '0';
								PCE <= '1';
								
								IRE <= '0';
								MDRE <= '0';
								MARE <= '0';
								AE <= '1'; 
								BE <= '0';
								SRE <= '0';
								MemWR <= '0';
								
								IorD <= '0';
								ALUSRCA <= '0';
								ALUSRCB <= "01";
								ALUCTRL <= "000";
								ASRC <= "00";
								BSRC <= "00";

								JZ <= '0';
								JV <= '0';
								JC <= '0';
								Pause <= '0';
								
								counter <= 0;
							end if;
				when "01110000" => --SUB #X
							if counter = 1 then -- cycle 2
								PCSRC <= '0';
								PCE <= '0';
								
								IRE <= '0';
								MDRE <= '1';
								MARE <= '0';
								AE <= '0'; 
								BE <= '0';
								SRE <= '0';
								MemWR <= '0';
								
								IorD <= '0';
								ALUSRCA <= '0';
								ALUSRCB <= "01";
								ALUCTRL <= "000";
								ASRC <= "00";
								BSRC <= "00";

								JZ <= '0';
								JV <= '0';
								JC <= '0';
								Pause <= '0';
								
								counter <= counter + 1;
							elsif counter = 2 then
								PCSRC <= '0';
								PCE <= '0';
								
								IRE <= '0';
								MDRE <= '0';
								MARE <= '0';
								AE <= '0'; 
								BE <= '0';
								SRE <= '1';
								MemWR <= '0';
								
								IorD <= '0';
								ALUSRCA <= '1';
								ALUSRCB <= "10";
								ALUCTRL <= "001";
								ASRC <= "00";
								BSRC <= "00";

								JZ <= '0';
								JV <= '0';
								JC <= '0';
								Pause <= '0';
								
								counter <= counter + 1;
							elsif counter = 3 then
								PCSRC <= '0';
								PCE <= '1';
								
								IRE <= '0';
								MDRE <= '0';
								MARE <= '0';
								AE <= '1'; 
								BE <= '0';
								SRE <= '0';
								MemWR <= '0';
								
								IorD <= '0';
								ALUSRCA <= '0';
								ALUSRCB <= "01";
								ALUCTRL <= "000";
								ASRC <= "00";
								BSRC <= "00";

								JZ <= '0';
								JV <= '0';
								JC <= '0';
								Pause <= '0';
								
								counter <= 0;
							end if;
				when "10000000" => --MUL X
							if counter = 1 then -- cycle 2
								PCSRC <= '0';
								PCE <= '0';
								
								IRE <= '0';
								MDRE <= '0';
								MARE <= '1';
								AE <= '0'; 
								BE <= '0';
								SRE <= '0';
								MemWR <= '0';
								
								IorD <= '0';
								ALUSRCA <= '0';
								ALUSRCB <= "01";
								ALUCTRL <= "000";
								ASRC <= "00";
								BSRC <= "00";

								JZ <= '0';
								JV <= '0';
								JC <= '0';
								Pause <= '0';
								
								counter <= counter + 1;
							elsif counter = 2 then
								PCSRC <= '0';
								PCE <= '0';
								
								IRE <= '0';
								MDRE <= '1';
								MARE <= '0';
								AE <= '0'; 
								BE <= '0';
								SRE <= '0';
								MemWR <= '0';
								
								IorD <= '1';
								ALUSRCA <= '0';
								ALUSRCB <= "01";
								ALUCTRL <= "000";
								ASRC <= "00";
								BSRC <= "00";

								JZ <= '0';
								JV <= '0';
								JC <= '0';
								Pause <= '0';
								
								counter <= counter + 1;
							elsif counter = 3 then
								PCSRC <= '0';
								PCE <= '0';
								
								IRE <= '0';
								MDRE <= '0';
								MARE <= '0';
								AE <= '0'; 
								BE <= '0';
								SRE <= '0';
								MemWR <= '0';
								
								IorD <= '0';
								ALUSRCA <= '0';
								ALUSRCB <= "00";
								ALUCTRL <= "000";
								ASRC <= "00";
								BSRC <= "00";

								JZ <= '0';
								JV <= '0';
								JC <= '0';
								Pause <= '0';
								
								counter <= counter + 1;
							elsif counter = 4 then
								PCSRC <= '0';
								PCE <= '1';
								
								IRE <= '0';
								MDRE <= '0';
								MARE <= '0';
								AE <= '1'; 
								BE <= '1';
								SRE <= '0';
								MemWR <= '0';
								
								IorD <= '0';
								ALUSRCA <= '0';
								ALUSRCB <= "01";
								ALUCTRL <= "000";
								ASRC <= "11";
								BSRC <= "11";

								JZ <= '0';
								JV <= '0';
								JC <= '0';
								Pause <= '0';
								
								counter <= 0;
							end if;
				when "10010000" => --MUL #X
							if counter = 1 then -- cycle 2
								PCSRC <= '0';
								PCE <= '0';
								
								IRE <= '0';
								MDRE <= '1';
								MARE <= '0';
								AE <= '0'; 
								BE <= '0';
								SRE <= '0';
								MemWR <= '0';
								
								IorD <= '0';
								ALUSRCA <= '0';
								ALUSRCB <= "01";
								ALUCTRL <= "000";
								ASRC <= "00";
								BSRC <= "00";

								JZ <= '0';
								JV <= '0';
								JC <= '0';
								Pause <= '0';
								
								counter <= counter + 1;
							elsif counter = 2 then
								PCSRC <= '0';
								PCE <= '0';
								
								IRE <= '0';
								MDRE <= '0';
								MARE <= '0';
								AE <= '0'; 
								BE <= '0';
								SRE <= '0';
								MemWR <= '0';
								
								IorD <= '0';
								ALUSRCA <= '0';
								ALUSRCB <= "00";
								ALUCTRL <= "000";
								ASRC <= "00";
								BSRC <= "00";

								JZ <= '0';
								JV <= '0';
								JC <= '0';
								Pause <= '0';
								
								counter <= counter + 1;
							elsif counter = 3 then
								PCSRC <= '0';
								PCE <= '1';
								
								IRE <= '0';
								MDRE <= '0';
								MARE <= '0';
								AE <= '1'; 
								BE <= '1';
								SRE <= '0';
								MemWR <= '0';
								
								IorD <= '0';
								ALUSRCA <= '0';
								ALUSRCB <= "01";
								ALUCTRL <= "000";
								ASRC <= "11";
								BSRC <= "11";

								JZ <= '0';
								JV <= '0';
								JC <= '0';
								Pause <= '0';
								
								counter <= 0;
							end if;
				when "10100000" => --SWAP
							if counter = 1 then -- cycle 2
								PCSRC <= '0';
								PCE <= '0';
								
								IRE <= '0';
								MDRE <= '0';
								MARE <= '0';
								AE <= '0'; 
								BE <= '0';
								SRE <= '1';
								MemWR <= '0';
								
								IorD <= '0';
								ALUSRCA <= '1';
								ALUSRCB <= "11";
								ALUCTRL <= "010";
								ASRC <= "00";
								BSRC <= "00";

								JZ <= '0';
								JV <= '0';
								JC <= '0';
								Pause <= '0';
								
								counter <= counter + 1;
							elsif counter = 2 then
								PCSRC <= '0';
								PCE <= '0';
								
								IRE <= '0';
								MDRE <= '0';
								MARE <= '0';
								AE <= '0'; 
								BE <= '1';
								SRE <= '1';
								MemWR <= '0';
								
								IorD <= '0';
								ALUSRCA <= '1';
								ALUSRCB <= "00";
								ALUCTRL <= "011";
								ASRC <= "00";
								BSRC <= "00";

								JZ <= '0';
								JV <= '0';
								JC <= '0';
								Pause <= '0';
								
								counter <= counter + 1;
							elsif counter = 3 then
								PCSRC <= '0';
								PCE <= '1';
								
								IRE <= '0';
								MDRE <= '0';
								MARE <= '0';
								AE <= '1'; 
								BE <= '0';
								SRE <= '0';
								MemWR <= '0';
								
								IorD <= '0';
								ALUSRCA <= '0';
								ALUSRCB <= "01";
								ALUCTRL <= "000";
								ASRC <= "00";
								BSRC <= "00";

								JZ <= '0';
								JV <= '0';
								JC <= '0';
								Pause <= '0';
								
								counter <= 0;
							end if;
				when "10110000" => -- Pause
						if counter = 1 then -- cycle 2
								PCSRC <= '0';
								PCE <= '0';
								
								IRE <= '0';
								MDRE <= '0';
								MARE <= '0';
								AE <= '0'; 
								BE <= '0';
								SRE <= '0';
								MemWR <= '0';
								
								IorD <= '0';
								ALUSRCA <= '0';
								ALUSRCB <= "01";
								ALUCTRL <= "000";
								ASRC <= "00";
								BSRC <= "00";

								JZ <= '0';
								JV <= '0';
								JC <= '0';
								Pause <= '1';
								
								counter <= counter + 1;
							elsif counter = 2 then -- cycle 3 
								PCSRC <= '0';
								PCE <= '1';
								
								IRE <= '0';
								MDRE <= '0';
								MARE <= '0';
								AE <= '0'; 
								BE <= '0';
								SRE <= '0';
								MemWR <= '0';
								
								IorD <= '0';
								ALUSRCA <= '0';
								ALUSRCB <= "01";
								ALUCTRL <= "000"; -- add
								ASRC <= "00";
								BSRC <= "00";

								JZ <= '0';
								JV <= '0';
								JC <= '0';
								Pause <= '0';
								
								counter <= 0;
							end if;
				when "11000000" => --JZ X
							if counter = 1 then -- cycle 2
								PCSRC <= '0';
								PCE <= '0';
								
								IRE <= '0';
								MDRE <= '1';
								MARE <= '0';
								AE <= '0'; 
								BE <= '0';
								SRE <= '1';
								MemWR <= '0';
								
								IorD <= '0';
								ALUSRCA <= '0';
								ALUSRCB <= "01";
								ALUCTRL <= "000";
								ASRC <= "00";
								BSRC <= "00";

								JZ <= '0';
								JV <= '0';
								JC <= '0';
								Pause <= '0';
								
								counter <= counter + 1;
							elsif counter = 2 then
								PCSRC <= '1';
								PCE <= '1';
								
								IRE <= '0';
								MDRE <= '0';
								MARE <= '0';
								AE <= '0'; 
								BE <= '0';
								SRE <= '0';
								MemWR <= '0';
								
								IorD <= '1';
								ALUSRCA <= '1';
								ALUSRCB <= "11";
								ALUCTRL <= "000";
								ASRC <= "00";
								BSRC <= "00";

								JZ <= '1';
								JV <= '0';
								JC <= '0';
								Pause <= '0';
								
								counter <= 0;
							end if;
				when "11010000" => --JC X
							if counter = 1 then -- cycle 2
								PCSRC <= '0';
								PCE <= '0';
								
								IRE <= '0';
								MDRE <= '1';
								MARE <= '0';
								AE <= '0'; 
								BE <= '0';
								SRE <= '1';
								MemWR <= '0';
								
								IorD <= '0';
								ALUSRCA <= '0';
								ALUSRCB <= "01";
								ALUCTRL <= "000";
								ASRC <= "00";
								BSRC <= "00";

								JZ <= '0';
								JV <= '0';
								JC <= '0';
								Pause <= '0';
								
								counter <= counter + 1;
							
							elsif counter = 2 then
								PCSRC <= '1';
								PCE <= '1';
								
								IRE <= '0';
								MDRE <= '0';
								MARE <= '0';
								AE <= '0'; 
								BE <= '0';
								SRE <= '0';
								MemWR <= '0';
								
								IorD <= '1';
								ALUSRCA <= '1';
								ALUSRCB <= "11";
								ALUCTRL <= "000";
								ASRC <= "00";
								BSRC <= "00";

								JZ <= '0';
								JV <= '0';
								JC <= '1';
								Pause <= '0';
								
								counter <= 0;
							end if;
				when "11100000" => --JV X
							if counter = 1 then -- cycle 2
								PCSRC <= '0';
								PCE <= '0';
								
								IRE <= '0';
								MDRE <= '1';
								MARE <= '0';
								AE <= '0'; 
								BE <= '0';
								SRE <= '1';
								MemWR <= '0';
								
								IorD <= '0';
								ALUSRCA <= '0';
								ALUSRCB <= "01";
								ALUCTRL <= "000";
								ASRC <= "00";
								BSRC <= "00";

								JZ <= '0';
								JV <= '0';
								JC <= '0';
								Pause <= '0';
								
								counter <= counter + 1;
							elsif counter = 2 then
								PCSRC <= '1';
								PCE <= '1';
								
								IRE <= '0';
								MDRE <= '0';
								MARE <= '0';
								AE <= '0'; 
								BE <= '0';
								SRE <= '0';
								MemWR <= '0';
								
								IorD <= '1';
								ALUSRCA <= '1';
								ALUSRCB <= "11";
								ALUCTRL <= "000";
								ASRC <= "00";
								BSRC <= "00";

								JZ <= '0';
								JV <= '1';
								JC <= '0';
								Pause <= '0';
								
								counter <= 0;
							end if;
					when others =>
								PCSRC <= '0';
								PCE <= '0';
								
								IRE <= '0';
								MDRE <= '0';
								MARE <= '0';
								AE <= '0'; 
								BE <= '0';
								SRE <= '0';
								MemWR <= '0';
								
								IorD <= '0';
								ALUSRCA <= '0';
								ALUSRCB <= "00";
								ALUCTRL <= "000";
								ASRC <= "00";
								BSRC <= "00";

								JZ <= '0';
								JV <= '0';
								JC <= '0';
								Pause <= '0';
								
								counter <= 0;
					end case;
				end if;
			end if;
	end if;
end if;
end process;
end CUBehavioral;



-- MAIN CODE BLOCK 
library IEEE;
use IEEE.STD_LOGIC_1164.ALL;
use IEEE.NUMERIC_STD.ALL;

Entity Microprocessor_8_bit is 
Port ( clk, Push_button: in std_logic;
	Load_mode: in std_logic := '0';
New_instruction: in std_logic_vector (7 downto 0):= "00000000";
PC_test_vec: out std_logic_vector (7 downto 0) := "00000000";
IR_test_vec: out std_logic_vector (7 downto 0) := "00000000";
MDR_test_vec: out std_logic_vector (7 downto 0) := "00000000";
MAR_test_vec: out std_logic_vector (7 downto 0) := "00000000";
Test_vec_1: out std_logic_vector (7 downto 0):= "00000000";
Test_vec_2: out std_logic_vector (7 downto 0):= "00000000"  );
End Microprocessor_8_bit; 

Architecture structure_main of Microprocessor_8_bit is 

Component CU is
	Port(Inst: in std_logic_vector (7 downto 0) :="00000000";
			Enable, load_mode: in std_logic :='0';
			CLK: in std_logic :='0';
			PCE, IRE, MDRE, MARE, AE, BE, SRE, PCreset, program: out std_logic :='0';
			MemWR, IorD, Pause, ALUSRCA, PCSRC, JZ, JC, JV: out std_logic :='0';
			ASRC, BSRC, ALUSRCB: out std_logic_vector (1 downto 0) :="00";
			ALUCTRL: out std_logic_vector (2 downto 0):="000");
end Component;

component multiplier is
Port(A: in std_logic_vector (7 downto 0):= x"00";
	  B: in std_logic_vector (7 downto 0) := x"00";
	  CLK: in std_logic := '0';
	  Z: out std_logic_vector (15 downto 0) := x"0000");
end component;

component mux_2 is
Port(a0: in std_logic_vector (7 downto 0):= x"00";
	  a1: in std_logic_vector (7 downto 0):= x"00";
	  sel: in std_logic := '0';
	  Z: out std_logic_vector (7 downto 0):= x"00");
end component;

component mux_4 is
Port(a0: in std_logic_vector (7 downto 0) := x"00";
	  a1: in std_logic_vector (7 downto 0) := x"00";
	  a2: in std_logic_vector (7 downto 0) := x"00";
	  a3: in std_logic_vector (7 downto 0) := x"00";
	  sel: in std_logic_vector (1 downto 0):= "00";
	  Z: out std_logic_vector (7 downto 0) := x"00");
end component;

Component DFF_fallingEdge is 
   port(
		Clk :in std_logic :='0';  
		reset: in std_logic :='0';
		D :in  std_logic :='0'; 
      Q : out std_logic :='0';   
		QN : out std_logic :='1' );
End component;

Component  DFF_risingEdge is 
   port(
		Clk :in std_logic:='0';  
		reset: in std_logic:='0';
		D :in  std_logic:='0'; 
      Q : out std_logic:='0';   
		QN : out std_logic :='1');
End component;

Component register_8_bit is 
port (clk: in std_logic :='0';
		reset: in std_logic :='0';
		En: in std_logic :='0';
		in_vec: in std_logic_vector (7 downto 0) := "00000000";
		out_vec: out std_logic_vector (7 downto 0) := "00000000");
end component;

Component  Memory_256x8 is
port(
 clk: in std_logic :='0';
 RW_en: in std_logic :='0'; 
 mem_addr: in std_logic_vector(7 downto 0); -- Address to write/read 
 mem_data_in: in std_logic_vector(7 downto 0); -- data that is to be writen to address 
 mem_data_out: out std_logic_vector(7 downto 0)); -- Data out
end component;

Component ALU_8_bit is 
 port ( A_in, B_in: in std_logic_vector (7 downto 0):="00000000";
			ALU_Ctrl: in std_logic_vector (2 downto 0):="000"; 
			Z_out: out std_logic_vector (7 downto 0):="00000000";
			ZF,CF,VF: out std_logic := '0');
 End component;


-- signals 
--Pause Circuit associated signals
Signal ff_1_Q_out: std_logic :='0';
Signal ff_1_QN_out: std_logic :='1';
Signal ff_2_Q_out: std_logic :='0';
Signal ff_2_QN_out: std_logic :='1';

-- PC Register associated signals 
Signal PC_Out: std_logic_vector (7 downto 0):= "00000000";
Signal PCEN: std_logic := '0';
Signal PC_reset: std_logic := '0';

-- MAR Register associated signals
Signal MAR_Out: std_logic_vector (7 downto 0):= "00000000";
Signal MAREN: std_logic := '0';
 
-- A Register associated signals 
Signal A_Out: std_logic_vector (7 downto 0):= "00000000";
Signal AEN: std_logic := '0';

-- B Register associated signals 
Signal B_Out: std_logic_vector (7 downto 0):= "00000000";
Signal BEN: std_logic := '0';

-- SR Register associated signals 
Signal SR_Out: std_logic_vector (7 downto 0):= "00000000";
Signal SREN: std_logic := '0';

-- IR associated signals 
Signal IR_out: std_logic_vector (7 downto 0) := "00000000";
Signal IREN: std_logic := '0';

-- MDR associated Signals 
Signal MDR_Out : std_logic_vector (7 downto 0):= "00000000";
Signal MDREN: std_logic := '0';

--CU signals
Signal pause: std_logic := '0';

-- memory associated signals 
Signal Mem_Out : std_logic_vector (7 downto 0):= "00000000";
Signal MemWR_sel: std_logic := '0';

--MUX selects
Signal program_sel: std_logic := '0';
Signal program_Out: std_logic_vector (7 downto 0):= "00000000";
Signal IorD_sel: std_logic := '0';
Signal IorD_Out: std_logic_vector (7 downto 0):= "00000000";
Signal ALUSRCA_sel: std_logic := '0';
Signal ALUSRCA_Out: std_logic_vector (7 downto 0):= "00000000";
Signal PCSRC_sel: std_logic := '0';
Signal PCSRC_Out: std_logic_vector (7 downto 0):= "00000000";
Signal ASRC_sel: std_logic_vector (1 downto 0) := "00";
Signal ASRC_Out: std_logic_vector (7 downto 0):= "00000000";
Signal BSRC_sel: std_logic_vector (1 downto 0) := "00";
Signal BSRC_Out: std_logic_vector (7 downto 0):= "00000000";
Signal ALUSRCB_sel: std_logic_vector (1 downto 0) := "00";
Signal ALUSRCB_Out: std_logic_vector (7 downto 0):= "00000000";
Signal jumpmux_sel: std_logic := '0';
Signal jumpmux_Out: std_logic_vector (7 downto 0):= "00000000";

--ALU signals
Signal ALUCTRL: std_logic_vector (2 downto 0) := "000";
Signal ALU_Out: std_logic_vector (7 downto 0) := "00000000";
Signal ZF: std_logic := '0';
Signal CF: std_logic := '0';
Signal VF: std_logic := '0';

--multiplier associated signals
Signal multiplier_Out: std_logic_vector (15 downto 0) := x"0000";

--Jump signals
Signal JZ_en: std_logic := '0';
Signal JC_en: std_logic := '0';
Signal JV_en: std_logic := '0';
signal jumptemp: std_logic_vector (2 downto 0) := "000";

Begin 
-- this is the left flip flop in our pause setup. 
Pause_FF_1:  DFF_fallingEdge port map (push_button, ff_1_Q_out, '1', ff_1_Q_out, ff_1_QN_out);

-- this is the right flip flop in our pause setup. 
Pause_FF_2: 	DFF_risingEdge port map (pause, ff_1_Q_out, '1', ff_2_Q_out, ff_2_QN_out);

-- this is the Control Unit 
CUlabel: CU port map(IR_Out, ff_2_QN_out, load_mode, CLK, PCEN, IREN, MDREN, MAREN, AEN, BEN, SREN, PC_reset, program_sel, MEMWR_sel, IorD_sel, pause, ALUSRCA_sel, PCSRC_sel, JZ_en, JC_en, JV_en, ASRC_sel, BSRC_sel, ALUSRCB_sel, ALUCTRL);

-- this is the PC Register 
PC_Register: register_8_bit port map ( clk, PC_reset, PCEN, PCSRC_Out, PC_Out);

-- the IorD 2x1Mux 
IorDMux: mux_2 port map ( PC_Out, MAR_Out, IorD_sel, IorD_Out);
	
-- the program register, used for writing code to mem(1) or data 
ProgramMux: mux_2 port map ( A_Out, New_instruction, program_sel, program_Out);

-- the Main Memory 256 x 8
Memory:  Memory_256x8 port map (clk,  MemWR_sel, IorD_Out, Program_Out, Mem_Out);

-- the Instruction Register 
IR_Register: register_8_bit port map (clk, '0', IREN, Mem_Out, IR_Out);

-- the Mem Data Register 
MDR_Register: register_8_bit port map (clk, '0', MDREN, Mem_Out, MDR_Out);

-- the Mem Address Register 
MAR_Register: register_8_bit port map (clk, '0', MAREN, Mem_Out, MAR_Out);

-- this is the A source Mux 4x1
A_Source: mux_4 port map ( SR_Out, MDR_Out, "00000000", Multiplier_Out(15 downto 8), ASRC_sel, ASRC_Out );

-- this is the B source MUX 4x1
B_Source: mux_4 port map ( SR_Out, MDR_Out, "00000000", multiplier_Out (7 downto 0), BSRC_sel, BSRC_Out);

-- the A Register 
A_Register: register_8_bit port map (clk, '0', AEN, ASRC_Out, A_Out);

-- the B Register 
B_Register: register_8_bit port map (clk, '0', BEN, BSRC_Out, B_Out);

-- ALU source A mux 2x1
ALU_SRC_A: mux_2 port map ( PC_Out,A_Out, ALUSRCA_sel, ALUSRCA_Out);

-- ALU source B mux 4x1
ALU_SRC_B: mux_4 port map ( B_Out, "00000001", MDR_Out, "00000000", ALUSRCB_sel, ALUSRCB_Out);

--ALU
ALU: ALU_8_bit port map(ALUSRCA_Out, ALUSRCB_Out, ALUCTRL, ALU_Out, ZF, CF, VF);

--section for jump control
jumptemp(0) <= ZF and JZ_en;
jumptemp(1) <= CF and JC_en;
jumptemp(2) <= VF and JV_en;
jumpmux_sel <= jumptemp(0) or jumptemp(1) or jumptemp(2);

jumpmux: mux_2 port map (SR_Out, MDR_Out, jumpmux_sel, jumpmux_Out);
--Multiplier
Multiplierlabel: multiplier port map (MDR_Out, B_Out, CLK, multiplier_Out);

-- the S Register 
SR_Register: register_8_bit port map (clk, '0', SREN, ALU_Out, SR_Out);

-- PC source mux 2x1
PC_SRC: mux_2 port map ( ALU_Out, jumpmux_Out, PCSRC_sel, PCSRC_Out);

Test_vec_1 <= A_Out;
Test_vec_2 <= B_Out;
PC_test_vec <= PC_Out;
IR_test_vec <= IR_Out;
MDR_test_vec <= MDR_Out;
MAR_test_vec <= MAR_Out;
End structure_main;