/*
Name 1: Ashkan Vafaee
UTEID 1: av28837
*/

/***************************************************************/
/*                                                             */
/*   LC-3b Simulator                                           */
/*                                                             */
/*   EE 460N                                                   */
/*   The University of Texas at Austin                         */
/*                                                             */
/***************************************************************/

#include <string.h>
#include <stdio.h>
#include <stdlib.h>

/***************************************************************/
/*                                                             */
/* Files:  ucode        Microprogram file                      */
/*         isaprogram   LC-3b machine language program file    */
/*                                                             */
/***************************************************************/

/***************************************************************/
/* These are the functions you'll have to write.               */
/***************************************************************/

void eval_micro_sequencer();
void cycle_memory();
void eval_bus_drivers();
void drive_bus();
void latch_datapath_values();

/***************************************************************/
/* A couple of useful definitions.                             */
/***************************************************************/
#define FALSE 0
#define TRUE  1

/***************************************************************/
/* Use this to avoid overflowing 16 bits on the bus.           */
/***************************************************************/
#define Low16bits(x) ((x) & 0xFFFF)

/***************************************************************/
/* Definition of the control store layout.                     */
/***************************************************************/
#define CONTROL_STORE_ROWS 64
#define INITIAL_STATE_NUMBER 18

/***************************************************************/
/* Definition of bit order in control store word.              */
/***************************************************************/
enum CS_BITS {
	IRD,
	COND1, COND0,
	J5, J4, J3, J2, J1, J0,
	LD_MAR,
	LD_MDR,
	LD_IR,
	LD_BEN,
	LD_REG,
	LD_CC,
	LD_PC,
	GATE_PC,
	GATE_MDR,
	GATE_ALU,
	GATE_MARMUX,
	GATE_SHF,
	PCMUX1, PCMUX0,
	DRMUX,
	SR1MUX,
	ADDR1MUX,
	ADDR2MUX1, ADDR2MUX0,
	MARMUX,
	ALUK1, ALUK0,
	MIO_EN,
	R_W,
	DATA_SIZE,
	LSHF1,
	CONTROL_STORE_BITS
} CS_BITS;

/***************************************************************/
/* Functions to get at the control bits.                       */
/***************************************************************/
int GetIRD(int *x) { return(x[IRD]); }
int GetCOND(int *x) { return((x[COND1] << 1) + x[COND0]); }
int GetJ(int *x) {
	return((x[J5] << 5) + (x[J4] << 4) +
		(x[J3] << 3) + (x[J2] << 2) +
		(x[J1] << 1) + x[J0]);
}
int GetLD_MAR(int *x) { return(x[LD_MAR]); }
int GetLD_MDR(int *x) { return(x[LD_MDR]); }
int GetLD_IR(int *x) { return(x[LD_IR]); }
int GetLD_BEN(int *x) { return(x[LD_BEN]); }
int GetLD_REG(int *x) { return(x[LD_REG]); }
int GetLD_CC(int *x) { return(x[LD_CC]); }
int GetLD_PC(int *x) { return(x[LD_PC]); }
int GetGATE_PC(int *x) { return(x[GATE_PC]); }
int GetGATE_MDR(int *x) { return(x[GATE_MDR]); }
int GetGATE_ALU(int *x) { return(x[GATE_ALU]); }
int GetGATE_MARMUX(int *x) { return(x[GATE_MARMUX]); }
int GetGATE_SHF(int *x) { return(x[GATE_SHF]); }
int GetPCMUX(int *x) { return((x[PCMUX1] << 1) + x[PCMUX0]); }
int GetDRMUX(int *x) { return(x[DRMUX]); }
int GetSR1MUX(int *x) { return(x[SR1MUX]); }
int GetADDR1MUX(int *x) { return(x[ADDR1MUX]); }
int GetADDR2MUX(int *x) { return((x[ADDR2MUX1] << 1) + x[ADDR2MUX0]); }
int GetMARMUX(int *x) { return(x[MARMUX]); }
int GetALUK(int *x) { return((x[ALUK1] << 1) + x[ALUK0]); }
int GetMIO_EN(int *x) { return(x[MIO_EN]); }
int GetR_W(int *x) { return(x[R_W]); }
int GetDATA_SIZE(int *x) { return(x[DATA_SIZE]); }
int GetLSHF1(int *x) { return(x[LSHF1]); }

/***************************************************************/
/* The control store rom.                                      */
/***************************************************************/
int CONTROL_STORE[CONTROL_STORE_ROWS][CONTROL_STORE_BITS];

/***************************************************************/
/* Main memory.                                                */
/***************************************************************/
/* MEMORY[A][0] stores the least significant byte of word at word address A
MEMORY[A][1] stores the most significant byte of word at word address A
There are two write enable signals, one for each byte. WE0 is used for
the least significant byte of a word. WE1 is used for the most significant
byte of a word. */

#define WORDS_IN_MEM    0x08000 
#define MEM_CYCLES      5
int MEMORY[WORDS_IN_MEM][2];
int MEM_CYCLE_COUNT = 1;			/* Keeps track of current memory cycle */

/***************************************************************/

/***************************************************************/

/***************************************************************/
/* LC-3b State info.                                           */
/***************************************************************/
#define LC_3b_REGS 8

int RUN_BIT;	/* run bit */
int BUS;	/* value of the bus */

typedef struct System_Latches_Struct {

	int PC,		/* program counter */
		MDR,	/* memory data register */
		MAR,	/* memory address register */
		IR,		/* instruction register */
		N,		/* n condition bit */
		Z,		/* z condition bit */
		P,		/* p condition bit */
		BEN;        /* ben register */

	int READY;	/* ready bit */
				/* The ready bit is also latched as you dont want the memory system to assert it
				at a bad point in the cycle*/

	int REGS[LC_3b_REGS]; /* register file. */

	int MICROINSTRUCTION[CONTROL_STORE_BITS]; /* The microintruction */

	int STATE_NUMBER; /* Current State Number - Provided for debugging */
} System_Latches;

/* Data Structure for Latch */

System_Latches CURRENT_LATCHES, NEXT_LATCHES;

/***************************************************************/
/* A cycle counter.                                            */
/***************************************************************/
int CYCLE_COUNT;

/***************************************************************/
/*                                                             */
/* Procedure : help                                            */
/*                                                             */
/* Purpose   : Print out a list of commands.                   */
/*                                                             */
/***************************************************************/
void help() {
	printf("----------------LC-3bSIM Help-------------------------\n");
	printf("go               -  run program to completion       \n");
	printf("run n            -  execute program for n cycles    \n");
	printf("mdump low high   -  dump memory from low to high    \n");
	printf("rdump            -  dump the register & bus values  \n");
	printf("?                -  display this help menu          \n");
	printf("quit             -  exit the program                \n\n");
}

/***************************************************************/
/*                                                             */
/* Procedure : cycle                                           */
/*                                                             */
/* Purpose   : Execute a cycle                                 */
/*                                                             */
/***************************************************************/
void cycle() {

	eval_micro_sequencer();
	cycle_memory();
	eval_bus_drivers();
	drive_bus();
	latch_datapath_values();

	CURRENT_LATCHES = NEXT_LATCHES;

	CYCLE_COUNT++;
}

/***************************************************************/
/*                                                             */
/* Procedure : run n                                           */
/*                                                             */
/* Purpose   : Simulate the LC-3b for n cycles.                 */
/*                                                             */
/***************************************************************/
void run(int num_cycles) {
	int i;

	if (RUN_BIT == FALSE) {
		printf("Can't simulate, Simulator is halted\n\n");
		return;
	}

	printf("Simulating for %d cycles...\n\n", num_cycles);
	for (i = 0; i < num_cycles; i++) {
		if (CURRENT_LATCHES.PC == 0x0000) {
			RUN_BIT = FALSE;
			printf("Simulator halted\n\n");
			break;
		}
		cycle();
	}
}

/***************************************************************/
/*                                                             */
/* Procedure : go                                              */
/*                                                             */
/* Purpose   : Simulate the LC-3b until HALTed.                 */
/*                                                             */
/***************************************************************/
void go() {
	if (RUN_BIT == FALSE) {
		printf("Can't simulate, Simulator is halted\n\n");
		return;
	}

	printf("Simulating...\n\n");
	while (CURRENT_LATCHES.PC != 0x0000)
		cycle();
	RUN_BIT = FALSE;
	printf("Simulator halted\n\n");
}

/***************************************************************/
/*                                                             */
/* Procedure : mdump                                           */
/*                                                             */
/* Purpose   : Dump a word-aligned region of memory to the     */
/*             output file.                                    */
/*                                                             */
/***************************************************************/
void mdump(FILE * dumpsim_file, int start, int stop) {
	int address; /* this is a byte address */

	printf("\nMemory content [0x%.4x..0x%.4x] :\n", start, stop);
	printf("-------------------------------------\n");
	for (address = (start >> 1); address <= (stop >> 1); address++)
		printf("  0x%.4x (%d) : 0x%.2x%.2x\n", address << 1, address << 1, MEMORY[address][1], MEMORY[address][0]);
	printf("\n");

	/* dump the memory contents into the dumpsim file */
	fprintf(dumpsim_file, "\nMemory content [0x%.4x..0x%.4x] :\n", start, stop);
	fprintf(dumpsim_file, "-------------------------------------\n");
	for (address = (start >> 1); address <= (stop >> 1); address++)
		fprintf(dumpsim_file, " 0x%.4x (%d) : 0x%.2x%.2x\n", address << 1, address << 1, MEMORY[address][1], MEMORY[address][0]);
	fprintf(dumpsim_file, "\n");
	fflush(dumpsim_file);
}

/***************************************************************/
/*                                                             */
/* Procedure : rdump                                           */
/*                                                             */
/* Purpose   : Dump current register and bus values to the     */
/*             output file.                                    */
/*                                                             */
/***************************************************************/
void rdump(FILE * dumpsim_file) {
	int k;

	printf("\nCurrent register/bus values :\n");
	printf("-------------------------------------\n");
	printf("Cycle Count  : %d\n", CYCLE_COUNT);
	printf("PC           : 0x%.4x\n", CURRENT_LATCHES.PC);
	printf("IR           : 0x%.4x\n", CURRENT_LATCHES.IR);
	printf("STATE_NUMBER : 0x%.4x\n\n", CURRENT_LATCHES.STATE_NUMBER);
	printf("BUS          : 0x%.4x\n", BUS);
	printf("MDR          : 0x%.4x\n", CURRENT_LATCHES.MDR);
	printf("MAR          : 0x%.4x\n", CURRENT_LATCHES.MAR);
	printf("CCs: N = %d  Z = %d  P = %d\n", CURRENT_LATCHES.N, CURRENT_LATCHES.Z, CURRENT_LATCHES.P);
	printf("Registers:\n");
	for (k = 0; k < LC_3b_REGS; k++)
		printf("%d: 0x%.4x\n", k, CURRENT_LATCHES.REGS[k]);
	printf("\n");

	/* dump the state information into the dumpsim file */
	fprintf(dumpsim_file, "\nCurrent register/bus values :\n");
	fprintf(dumpsim_file, "-------------------------------------\n");
	fprintf(dumpsim_file, "Cycle Count  : %d\n", CYCLE_COUNT);
	fprintf(dumpsim_file, "PC           : 0x%.4x\n", CURRENT_LATCHES.PC);
	fprintf(dumpsim_file, "IR           : 0x%.4x\n", CURRENT_LATCHES.IR);
	fprintf(dumpsim_file, "STATE_NUMBER : 0x%.4x\n\n", CURRENT_LATCHES.STATE_NUMBER);
	fprintf(dumpsim_file, "BUS          : 0x%.4x\n", BUS);
	fprintf(dumpsim_file, "MDR          : 0x%.4x\n", CURRENT_LATCHES.MDR);
	fprintf(dumpsim_file, "MAR          : 0x%.4x\n", CURRENT_LATCHES.MAR);
	fprintf(dumpsim_file, "CCs: N = %d  Z = %d  P = %d\n", CURRENT_LATCHES.N, CURRENT_LATCHES.Z, CURRENT_LATCHES.P);
	fprintf(dumpsim_file, "Registers:\n");
	for (k = 0; k < LC_3b_REGS; k++)
		fprintf(dumpsim_file, "%d: 0x%.4x\n", k, CURRENT_LATCHES.REGS[k]);
	fprintf(dumpsim_file, "\n");
	fflush(dumpsim_file);
}

/***************************************************************/
/*                                                             */
/* Procedure : get_command                                     */
/*                                                             */
/* Purpose   : Read a command from standard input.             */
/*                                                             */
/***************************************************************/
void get_command(FILE * dumpsim_file) {
	char buffer[20];
	int start, stop, cycles;

	printf("LC-3b-SIM> ");

	scanf("%s", buffer);
	printf("\n");

	switch (buffer[0]) {
	case 'G':
	case 'g':
		go();
		break;

	case 'M':
	case 'm':
		scanf("%i %i", &start, &stop);
		mdump(dumpsim_file, start, stop);
		break;

	case '?':
		help();
		break;
	case 'Q':
	case 'q':
		printf("Bye.\n");
		exit(0);

	case 'R':
	case 'r':
		if (buffer[1] == 'd' || buffer[1] == 'D')
			rdump(dumpsim_file);
		else {
			scanf("%d", &cycles);
			run(cycles);
		}
		break;

	default:
		printf("Invalid Command\n");
		break;
	}
}

/***************************************************************/
/*                                                             */
/* Procedure : init_control_store                              */
/*                                                             */
/* Purpose   : Load microprogram into control store ROM        */
/*                                                             */
/***************************************************************/
void init_control_store(char *ucode_filename) {
	FILE *ucode;
	int i, j, index;
	char line[200];

	printf("Loading Control Store from file: %s\n", ucode_filename);

	/* Open the micro-code file. */
	if ((ucode = fopen(ucode_filename, "r")) == NULL) {
		printf("Error: Can't open micro-code file %s\n", ucode_filename);
		exit(-1);
	}

	/* Read a line for each row in the control store. */
	for (i = 0; i < CONTROL_STORE_ROWS; i++) {
		if (fscanf(ucode, "%[^\n]\n", line) == EOF) {
			printf("Error: Too few lines (%d) in micro-code file: %s\n",
				i, ucode_filename);
			exit(-1);
		}

		/* Put in bits one at a time. */
		index = 0;

		for (j = 0; j < CONTROL_STORE_BITS; j++) {
			/* Needs to find enough bits in line. */
			if (line[index] == '\0') {
				printf("Error: Too few control bits in micro-code file: %s\nLine: %d\n",
					ucode_filename, i);
				exit(-1);
			}
			if (line[index] != '0' && line[index] != '1') {
				printf("Error: Unknown value in micro-code file: %s\nLine: %d, Bit: %d\n",
					ucode_filename, i, j);
				exit(-1);
			}

			/* Set the bit in the Control Store. */
			CONTROL_STORE[i][j] = (line[index] == '0') ? 0 : 1;
			index++;
		}

		/* Warn about extra bits in line. */
		if (line[index] != '\0')
			printf("Warning: Extra bit(s) in control store file %s. Line: %d\n",
				ucode_filename, i);
	}
	printf("\n");
}

/************************************************************/
/*                                                          */
/* Procedure : init_memory                                  */
/*                                                          */
/* Purpose   : Zero out the memory array                    */
/*                                                          */
/************************************************************/
void init_memory() {
	int i;

	for (i = 0; i < WORDS_IN_MEM; i++) {
		MEMORY[i][0] = 0;
		MEMORY[i][1] = 0;
	}
}

/**************************************************************/
/*                                                            */
/* Procedure : load_program                                   */
/*                                                            */
/* Purpose   : Load program and service routines into mem.    */
/*                                                            */
/**************************************************************/
void load_program(char *program_filename) {
	FILE * prog;
	int ii, word, program_base;

	/* Open program file. */
	prog = fopen(program_filename, "r");
	if (prog == NULL) {
		printf("Error: Can't open program file %s\n", program_filename);
		exit(-1);
	}

	/* Read in the program. */
	if (fscanf(prog, "%x\n", &word) != EOF)
		program_base = word >> 1;
	else {
		printf("Error: Program file is empty\n");
		exit(-1);
	}

	ii = 0;
	while (fscanf(prog, "%x\n", &word) != EOF) {
		/* Make sure it fits. */
		if (program_base + ii >= WORDS_IN_MEM) {
			printf("Error: Program file %s is too long to fit in memory. %x\n",
				program_filename, ii);
			exit(-1);
		}

		/* Write the word to memory array. */
		MEMORY[program_base + ii][0] = word & 0x00FF;
		MEMORY[program_base + ii][1] = (word >> 8) & 0x00FF;
		ii++;
	}

	if (CURRENT_LATCHES.PC == 0) CURRENT_LATCHES.PC = (program_base << 1);

	printf("Read %d words from program into memory.\n\n", ii);
}

/***************************************************************/
/*                                                             */
/* Procedure : initialize                                      */
/*                                                             */
/* Purpose   : Load microprogram and machine language program  */
/*             and set up initial state of the machine.        */
/*                                                             */
/***************************************************************/
void initialize(char *ucode_filename, char *program_filename, int num_prog_files) {
	int i;
	init_control_store(ucode_filename);

	init_memory();
	for (i = 0; i < num_prog_files; i++) {
		load_program(program_filename);
		while (*program_filename++ != '\0');
	}
	CURRENT_LATCHES.Z = 1;
	CURRENT_LATCHES.STATE_NUMBER = INITIAL_STATE_NUMBER;
	memcpy(CURRENT_LATCHES.MICROINSTRUCTION, CONTROL_STORE[INITIAL_STATE_NUMBER], sizeof(int)*CONTROL_STORE_BITS);

	NEXT_LATCHES = CURRENT_LATCHES;

	RUN_BIT = TRUE;
}

/***************************************************************/
/*                                                             */
/* Procedure : main                                            */
/*                                                             */
/***************************************************************/
int main(int argc, char *argv[]) {
	FILE * dumpsim_file;

	/* Error Checking */
	if (argc < 3) {
		printf("Error: usage: %s <micro_code_file> <program_file_1> <program_file_2> ...\n",
			argv[0]);
		exit(1);
	}

	printf("LC-3b Simulator\n\n");

	initialize(argv[1], argv[2], argc - 2);

	if ((dumpsim_file = fopen("dumpsim", "w")) == NULL) {
		printf("Error: Can't open dumpsim file\n");
		exit(-1);
	}

	while (1)
		get_command(dumpsim_file);

}

/***************************************************************/
/* Do not modify the above code.
You are allowed to use the following global variables in your
code. These are defined above.

CONTROL_STORE
MEMORY
BUS

CURRENT_LATCHES
NEXT_LATCHES

You may define your own local/global variables and functions.
You may use the functions to get at the control bits defined
above.

Begin your code here 	  			       */
/***************************************************************/


void eval_micro_sequencer() {

	/*
	* Evaluate the address of the next state according to the
	* micro sequencer logic. Latch the next microinstruction.
	*/


	/* If IRD SET -> Next State = IR[15:12] */
	if (CONTROL_STORE[CURRENT_LATCHES.STATE_NUMBER][IRD]) {
		NEXT_LATCHES.STATE_NUMBER = CURRENT_LATCHES.IR;
		
		for (int i = 0; i < CONTROL_STORE_BITS; i++) {
			NEXT_LATCHES.MICROINSTRUCTION[i] = CONTROL_STORE[CURRENT_LATCHES.IR][i];
		}
	}

	else {
		int state = 0;
		int condition1 = CURRENT_LATCHES.MICROINSTRUCTION[COND1];
		int	condition0 = CURRENT_LATCHES.MICROINSTRUCTION[COND0];
		int branch = CURRENT_LATCHES.BEN;
		int R = CURRENT_LATCHES.READY;
		int IR11 = (CURRENT_LATCHES.IR & 0b100000000000) >> 11;

		/* Evaluating Next State Based on J bits and BEN, R, IR[11] */
		state |= (CURRENT_LATCHES.MICROINSTRUCTION[J5] << 5);
		state |= (CURRENT_LATCHES.MICROINSTRUCTION[J4] << 4);
		state |= (CURRENT_LATCHES.MICROINSTRUCTION[J3] << 3);
		state |= ((CURRENT_LATCHES.MICROINSTRUCTION[J2] & branch & condition1 & !condition0 ) << 2);
		state |= ((CURRENT_LATCHES.MICROINSTRUCTION[J1] & R & !condition1 & condition0) << 1);
		state |= ((CURRENT_LATCHES.MICROINSTRUCTION[J0] & IR11 & condition1 & condition1 ) << 0);

		NEXT_LATCHES.STATE_NUMBER = state;
		for (int i = 0; i < CONTROL_STORE_BITS; i++) {
			NEXT_LATCHES.MICROINSTRUCTION[i] = CONTROL_STORE[state][i];
		}
	}
}


void cycle_memory() {
	/*
	* This function emulates memory and the WE logic.
	* Keep track of which cycle of MEMEN we are dealing with.
	* If fourth, we need to latch Ready bit at the end of
	* cycle to prepare microsequencer for the fifth cycle.
	*/

	/* Only Increment */
	if (CURRENT_LATCHES.MICROINSTRUCTION[MIO_EN]) {
		
		/* Setting Ready Bit */
		if (MEM_CYCLE_COUNT == 4) {
			NEXT_LATCHES.READY = 1;
			MEM_CYCLE_COUNT++;

		}
		/* Clearing Ready Bit */
		else if (MEM_CYCLE_COUNT == 5) {
			MEM_CYCLE_COUNT = 1;
			NEXT_LATCHES.READY = 0;
		}

		else {
			MEM_CYCLE_COUNT++;
		}
	}

}

enum {
	BaseR = 0b111000000, IR11_9 = 0b111000000000, IR8_6 = 0b111000000, offset6 = 0b111111, PCoffset9 = 0b111111111, PCoffset11 = 0b11111111111,
	check_sign_offset6 = 0b100000, check_sign_PCoffset9 = 0b100000000, check_sign_PCoffset11 = 0b10000000000,
	sign_extend_offset6 = 0xFFE0, sign_extend_PCoffset9 = 0xFF, sign_extend_PCoffset11 = 0xFC00,
	offset8 = 11111111, SHF = 0b110000, amount4 = 4, sign_bit16 = 0x8000, sign_bit6 = 0b100000, imm5 = 0b11111, reg_bits = 0b111,
	sign_bit8 = 0x80, sign_extend8 = 0xFF00, sign_extend16 = 0xFFFF0000, n_bit = 0b100000000000, z_bit = 0b10000000000, p_bit = 0b1000000000

};


/* Determines which gate is driving the bus */
int GATE_MARMUX_VALUE, GATE_PC_VALUE, GATE_ALU_VALUE, GATE_SHF_VALUE, GATE_MDR_VALUE;

void eval_bus_drivers() {



	/*
	* Datapath routine emulating operations before driving the bus.
	* Evaluate the input of tristate drivers
	*             Gate_MARMUX,
	*		 Gate_PC,
	*		 Gate_ALU,
	*		 Gate_SHF,
	*		 Gate_MDR.
	*/

	int ADDR1MUX_VALUE, ADDR2MUX_VALUE, PLUS_ADDER_VALUE, SR1_VALUE, SR2_VALUE, SR2MUX_VALUE;
	
	/* If 0, IR[11:9] indicates register */
	if (CURRENT_LATCHES.MICROINSTRUCTION[SR1MUX] == 0) {
		SR1_VALUE = CURRENT_LATCHES.REGS[(CURRENT_LATCHES.IR & IR11_9) >> 9];
	}

	/* If 1, IR[8:6] indicates register */
	else {
		SR1_VALUE = CURRENT_LATCHES.REGS[(CURRENT_LATCHES.IR & IR8_6) >> 6];
	}


	/* If 0, select PC */
	if (CURRENT_LATCHES.MICROINSTRUCTION[ADDR1MUX] == 0) {
		ADDR1MUX_VALUE = CURRENT_LATCHES.PC;
	}

	/* If 1, select (SR1_VALUE) */
	else {
		ADDR1MUX_VALUE = SR1_VALUE;
	}


	/* If 0, then value is 0*/
	if (((CURRENT_LATCHES.MICROINSTRUCTION[ADDR2MUX1] << 1) | CURRENT_LATCHES.MICROINSTRUCTION[ADDR2MUX0]) == 0) {
		ADDR2MUX_VALUE = 0;
	}
	else if (((CURRENT_LATCHES.MICROINSTRUCTION[ADDR2MUX1] << 1) | CURRENT_LATCHES.MICROINSTRUCTION[ADDR2MUX0]) == 1) {
		ADDR2MUX_VALUE = CURRENT_LATCHES.IR & offset6;
		ADDR2MUX_VALUE = (ADDR2MUX_VALUE & check_sign_offset6) ? (ADDR2MUX_VALUE | sign_extend_offset6) : ADDR2MUX_VALUE;
	}
	else if (((CURRENT_LATCHES.MICROINSTRUCTION[ADDR2MUX1] << 1) | CURRENT_LATCHES.MICROINSTRUCTION[ADDR2MUX0]) == 2) {
		ADDR2MUX_VALUE = CURRENT_LATCHES.IR & PCoffset9;
		ADDR2MUX_VALUE = (ADDR2MUX_VALUE & check_sign_PCoffset9) ? (ADDR2MUX_VALUE | sign_extend_PCoffset9) : ADDR2MUX_VALUE;
	}
	else if (((CURRENT_LATCHES.MICROINSTRUCTION[ADDR2MUX1] << 1) | CURRENT_LATCHES.MICROINSTRUCTION[ADDR2MUX0]) == 3) {
		ADDR2MUX_VALUE = CURRENT_LATCHES.IR & PCoffset11;
		ADDR2MUX_VALUE = (ADDR2MUX_VALUE & check_sign_PCoffset11) ? (ADDR2MUX_VALUE | sign_extend_PCoffset11) : ADDR2MUX_VALUE;

	}

	if (CURRENT_LATCHES.MICROINSTRUCTION[LSHF1]) {
		ADDR2MUX_VALUE = ADDR2MUX_VALUE << 1;
	}

	PLUS_ADDER_VALUE = ADDR2MUX_VALUE + ADDR1MUX_VALUE;


	/* SETTING GATE_MARMUX_VALUE */

	/* If 0 select LSHF(ZEXT[IR[7:0]],1)*/
	if (CURRENT_LATCHES.MICROINSTRUCTION[MARMUX] == 0) {
		GATE_MARMUX_VALUE = (CURRENT_LATCHES.IR & offset8) << 1;
	}
	/* If 1 select result from PLUSMUX_VALUE */
	else {
		GATE_MARMUX_VALUE = PLUS_ADDER_VALUE;
	}

	/* SETTING GATE_PC_VALUE */
	GATE_PC_VALUE = CURRENT_LATCHES.PC;

	/* SETTING  GATE_SHF_VALUE */
	int mask;

	if (((CURRENT_LATCHES.IR & SHF) >> amount4) == 0) {				/* Logical Shift Left */
		GATE_SHF_VALUE = SR1_VALUE << (CURRENT_LATCHES.IR & amount4);
	}
	else if (((CURRENT_LATCHES.IR & SHF) >> amount4) == 0) {		/* Logical Shift Right */
		mask = 0xFFFF;

		for (int i = 0; i < (CURRENT_LATCHES.IR & amount4); i++) {
			mask = mask >> 1;
		}

		GATE_SHF_VALUE = SR1_VALUE >> (CURRENT_LATCHES.IR & amount4);
		GATE_SHF_VALUE = GATE_SHF_VALUE & mask;
	}

	else if (((CURRENT_LATCHES.IR & SHF) >> amount4) == 0) {		/* Arithmetic Shift Right */
		
		/* Preserving sign bit if it's a 1 */
		if (SR1_VALUE & sign_bit16) {
			mask = 0x8000;

			for (int i = 0; i < (CURRENT_LATCHES.IR & amount4); i++) {
				mask = mask >> 1;
				mask = mask | 0x8000;
			}

			GATE_SHF_VALUE = SR1_VALUE >> (CURRENT_LATCHES.IR & amount4);
			GATE_SHF_VALUE = GATE_SHF_VALUE | mask;
		}

		/* Preserving sign bit if it's a 0 */
		else {
			mask = 0xFFFF;

			for (int i = 0; i < (CURRENT_LATCHES.IR & amount4); i++) {
				mask = mask >> 1;
			}

			GATE_SHF_VALUE = SR1_VALUE >> (CURRENT_LATCHES.IR & amount4);
			GATE_SHF_VALUE = GATE_SHF_VALUE & mask;

		}
	}

	/* Setting GATE_ALU_VALUE */
	if (CURRENT_LATCHES.IR & sign_bit6) {
		SR2_VALUE = CURRENT_LATCHES.IR & imm5;
	}
	else {
		SR2_VALUE = CURRENT_LATCHES.REGS[CURRENT_LATCHES.IR & reg_bits];
	}

	/* Sign extending SR1 and SR2 as needed*/
	SR1_VALUE = SR1_VALUE & sign_bit16 ? SR1_VALUE | sign_extend16 : SR1_VALUE;
	SR2_VALUE = SR2_VALUE & sign_bit16 ? SR2_VALUE | sign_extend16 : SR2_VALUE;

	if ((CURRENT_LATCHES.MICROINSTRUCTION[ALUK1] << 1 | CURRENT_LATCHES.MICROINSTRUCTION[ALUK0]) == 0) {		/* ADD */
		GATE_ALU_VALUE = SR1_VALUE + SR2_VALUE;
		GATE_ALU_VALUE = Low16bits(GATE_ALU_VALUE);

	}
	else if ((CURRENT_LATCHES.MICROINSTRUCTION[ALUK1] << 1 | CURRENT_LATCHES.MICROINSTRUCTION[ALUK0]) == 1) {	/* AND */
		GATE_ALU_VALUE = SR1_VALUE & SR2_VALUE;
		GATE_ALU_VALUE = Low16bits(GATE_ALU_VALUE);

	}

	else if ((CURRENT_LATCHES.MICROINSTRUCTION[ALUK1] << 1 | CURRENT_LATCHES.MICROINSTRUCTION[ALUK0]) == 2) {	/* XOR */
		GATE_ALU_VALUE = SR1_VALUE ^ SR2_VALUE;
		GATE_ALU_VALUE = Low16bits(GATE_ALU_VALUE);
	}

	else if ((CURRENT_LATCHES.MICROINSTRUCTION[ALUK1] << 1 | CURRENT_LATCHES.MICROINSTRUCTION[ALUK0]) == 3) {	/* PASSA */
		GATE_ALU_VALUE = SR1_VALUE;
	}

	/* SETTING GATE_MDR_VALUE */
	if (CURRENT_LATCHES.MICROINSTRUCTION[DATA_SIZE] == 0 && ((CURRENT_LATCHES.MAR & 0b1) == 0)) {		/* Byte load from even address */
		GATE_MDR_VALUE = CURRENT_LATCHES.MDR & 0xFF;
		GATE_MDR_VALUE = GATE_MDR_VALUE & sign_bit8 ? GATE_MDR_VALUE | sign_extend8 : GATE_MDR_VALUE;
	}

	else if (CURRENT_LATCHES.MICROINSTRUCTION[DATA_SIZE] == 0 && ((CURRENT_LATCHES.MAR & 0b1) == 1)) {	/* Byte load from odd address */
		GATE_MDR_VALUE = (CURRENT_LATCHES.MDR >> 8) & 0xFF;
		GATE_MDR_VALUE = GATE_MDR_VALUE & sign_bit8 ? GATE_MDR_VALUE | sign_extend8 : GATE_MDR_VALUE;
	}

	else if (CURRENT_LATCHES.MICROINSTRUCTION[DATA_SIZE] == 1 && ((CURRENT_LATCHES.MAR & 0b1) == 0)) {	/* Word load from even address */
		GATE_MDR_VALUE = CURRENT_LATCHES.MDR;
	}


}


void drive_bus() {

	/*
	* Datapath routine for driving the bus from one of the 5 possible
	* tristate drivers.
	*/

	if (CURRENT_LATCHES.MICROINSTRUCTION[GATE_MARMUX]) {
		BUS = GATE_MARMUX_VALUE;
	}
	else if (CURRENT_LATCHES.MICROINSTRUCTION[GATE_PC]) {
		BUS = GATE_PC_VALUE;
	}
	else if (CURRENT_LATCHES.MICROINSTRUCTION[GATE_ALU]) {
		BUS = GATE_ALU_VALUE;
	}
	else if (CURRENT_LATCHES.MICROINSTRUCTION[GATE_SHF]) {
		BUS = GATE_SHF_VALUE;
	}
	else if (CURRENT_LATCHES.MICROINSTRUCTION[GATE_MDR]) {
		BUS = GATE_MDR_VALUE;
	}


}


void latch_datapath_values() {

	/*
	* Datapath routine for computing all functions that need to latch
	* values in the data path at the end of this cycle.  Some values
	* require sourcing the bus; therefore, this routine has to come
	* after drive_bus.
	*/



	/* Memory Store */
	if (CURRENT_LATCHES.MICROINSTRUCTION[MIO_EN] && CURRENT_LATCHES.MICROINSTRUCTION[R_W]) {
		if (CURRENT_LATCHES.MICROINSTRUCTION[DATA_SIZE] == 0 && ((CURRENT_LATCHES.MAR & 0b1) == 0)) {		/* Byte store to even address */
			MEMORY[CURRENT_LATCHES.MAR >> 1][0] = CURRENT_LATCHES.MDR & 0xFF;

		}

		else if (CURRENT_LATCHES.MICROINSTRUCTION[DATA_SIZE] == 0 && ((CURRENT_LATCHES.MAR & 0b1) == 1)) {	/* Byte store to odd address */
			MEMORY[CURRENT_LATCHES.MAR >> 1][1] = (CURRENT_LATCHES.MDR >> 8) & 0xFF;

		}

		else if (CURRENT_LATCHES.MICROINSTRUCTION[DATA_SIZE] == 1 && ((CURRENT_LATCHES.MAR & 0b1) == 0)) {	/* Word store to even address */
			MEMORY[CURRENT_LATCHES.MAR >> 1][0] = CURRENT_LATCHES.MDR & 0xFF;
			MEMORY[CURRENT_LATCHES.MAR >> 1][1] = (CURRENT_LATCHES.MDR >> 8) & 0xFF;
		}

	}




	/* Load MAR */
	if (CURRENT_LATCHES.MICROINSTRUCTION[LD_MAR]) {
		NEXT_LATCHES.MAR = BUS;
	}

	/* Load MDR */
	if (CURRENT_LATCHES.MICROINSTRUCTION[LD_MDR]) {

		/* Select from BUS*/
		if (CURRENT_LATCHES.MICROINSTRUCTION[MIO_EN] == 0) {			

			if (CURRENT_LATCHES.MICROINSTRUCTION[DATA_SIZE] == 0 && ((CURRENT_LATCHES.MAR & 0b1) == 0)) {		/* Byte store to even address */
				NEXT_LATCHES.MDR = BUS & 0xFF;
				NEXT_LATCHES.MDR = NEXT_LATCHES.MDR & sign_bit8 ? NEXT_LATCHES.MDR | sign_extend8 : NEXT_LATCHES.MDR;
			}

			else if (CURRENT_LATCHES.MICROINSTRUCTION[DATA_SIZE] == 0 && ((CURRENT_LATCHES.MAR & 0b1) == 1)) {	/* Byte store to odd address */
				NEXT_LATCHES.MDR = (BUS >> 8) & 0xFF;
				NEXT_LATCHES.MDR = NEXT_LATCHES.MDR & sign_bit8 ? NEXT_LATCHES.MDR | sign_extend8 : NEXT_LATCHES.MDR;
			}

			else if (CURRENT_LATCHES.MICROINSTRUCTION[DATA_SIZE] == 1 && ((CURRENT_LATCHES.MAR & 0b1) == 0)) {	/* Word store to even address */
				NEXT_LATCHES.MDR = BUS;
			}


			/* Select From Memory*/
			else {

				if (CURRENT_LATCHES.MICROINSTRUCTION[DATA_SIZE] == 0 && ((CURRENT_LATCHES.MAR & 0b1) == 0)) {		/* Byte store to even address */
					NEXT_LATCHES.MDR = MEMORY[CURRENT_LATCHES.MAR >> 1][0];
					NEXT_LATCHES.MDR = NEXT_LATCHES.MDR & sign_bit8 ? NEXT_LATCHES.MDR | sign_extend8 : NEXT_LATCHES.MDR;

				}

				else if (CURRENT_LATCHES.MICROINSTRUCTION[DATA_SIZE] == 0 && ((CURRENT_LATCHES.MAR & 0b1) == 1)) {	/* Byte store to odd address */
					NEXT_LATCHES.MDR = MEMORY[CURRENT_LATCHES.MAR >> 1][1];
					NEXT_LATCHES.MDR = NEXT_LATCHES.MDR & sign_bit8 ? NEXT_LATCHES.MDR | sign_extend8 : NEXT_LATCHES.MDR;


				}

				else if (CURRENT_LATCHES.MICROINSTRUCTION[DATA_SIZE] == 1 && ((CURRENT_LATCHES.MAR & 0b1) == 0)) {	/* Word store to even address */
					NEXT_LATCHES.MDR = (MEMORY[CURRENT_LATCHES.MAR >> 1][1] << 8) | MEMORY[CURRENT_LATCHES.MAR >> 1][0];
				}
			}
		}
	}

	/* Load IR */
	if (CURRENT_LATCHES.MICROINSTRUCTION[LD_IR]) {
		NEXT_LATCHES.IR = BUS;
	}



	/* Load BEN */
	if (CURRENT_LATCHES.MICROINSTRUCTION[LD_BEN]) {
		NEXT_LATCHES.BEN = ((CURRENT_LATCHES.IR & n_bit) & CURRENT_LATCHES.N) + ((CURRENT_LATCHES.IR & z_bit) & CURRENT_LATCHES.Z) + ((CURRENT_LATCHES.IR & p_bit) & CURRENT_LATCHES.P);
	}

	/* Load Registers */
	if (CURRENT_LATCHES.MICROINSTRUCTION[LD_REG]) {

		/* Select register from IR[11:9] */
		if (CURRENT_LATCHES.MICROINSTRUCTION[DRMUX] == 0) {
			NEXT_LATCHES.REGS[(CURRENT_LATCHES.IR & IR11_9) >> 9] = BUS;
		}
		/* Select register 7 */
		else {
			NEXT_LATCHES.REGS[7] = BUS;
		}

	}


	/* Load CC bits*/
	if (CURRENT_LATCHES.MICROINSTRUCTION[LD_CC]) {
		int temp = BUS;
		temp = temp & sign_bit16 ? temp | sign_extend16 : temp;

		if (temp > 0) {
			NEXT_LATCHES.N = 0;
			NEXT_LATCHES.Z = 0;
			NEXT_LATCHES.P = 1;

		}
		else if (temp == 0) {
			NEXT_LATCHES.N = 0;
			NEXT_LATCHES.Z = 1;
			NEXT_LATCHES.P = 0;
		}

		else if (temp < 0) {
			NEXT_LATCHES.N = 1;
			NEXT_LATCHES.P = 0;
			NEXT_LATCHES.Z = 0;
		}

	}

	/* Load PC */
	if (CURRENT_LATCHES.MICROINSTRUCTION[LD_PC]) {


		/* Select PC + 2*/
		if (((CURRENT_LATCHES.MICROINSTRUCTION[PCMUX1] << 1) | CURRENT_LATCHES.MICROINSTRUCTION[PCMUX0]) == 0) {
			NEXT_LATCHES.PC = CURRENT_LATCHES.PC + 2;

		}

		/* Select BUS */
		if (((CURRENT_LATCHES.MICROINSTRUCTION[PCMUX1] << 1) | CURRENT_LATCHES.MICROINSTRUCTION[PCMUX0]) == 1) {
			NEXT_LATCHES.PC = BUS;

		}

		/* Select Result from ADDER */
		if (((CURRENT_LATCHES.MICROINSTRUCTION[PCMUX1] << 1) | CURRENT_LATCHES.MICROINSTRUCTION[PCMUX0]) == 2) {
			int ADDR1MUX_VALUE, ADDR2MUX_VALUE, SR1_VALUE;
			
			/* If 0, IR[11:9] indicates register */
			if (CURRENT_LATCHES.MICROINSTRUCTION[SR1MUX] == 0) {
				SR1_VALUE = CURRENT_LATCHES.REGS[(CURRENT_LATCHES.IR & IR11_9) >> 9];
			}

			/* If 1, IR[8:6] indicates register */
			else {
				SR1_VALUE = CURRENT_LATCHES.REGS[(CURRENT_LATCHES.IR & IR8_6) >> 6];
			}


			/* If 0, select PC */
			if (CURRENT_LATCHES.MICROINSTRUCTION[ADDR1MUX] == 0) {
				ADDR1MUX_VALUE = CURRENT_LATCHES.PC;
			}

			/* If 1, select (SR1_VALUE) */
			else {
				ADDR1MUX_VALUE = SR1_VALUE;
			}


			/* If 0, then value is 0*/
			if (((CURRENT_LATCHES.MICROINSTRUCTION[ADDR2MUX1] << 1) | CURRENT_LATCHES.MICROINSTRUCTION[ADDR2MUX0]) == 0) {
				ADDR2MUX_VALUE = 0;
			}
			else if (((CURRENT_LATCHES.MICROINSTRUCTION[ADDR2MUX1] << 1) | CURRENT_LATCHES.MICROINSTRUCTION[ADDR2MUX0]) == 1) {
				ADDR2MUX_VALUE = CURRENT_LATCHES.IR & offset6;
				ADDR2MUX_VALUE = (ADDR2MUX_VALUE & check_sign_offset6) ? (ADDR2MUX_VALUE | sign_extend_offset6) : ADDR2MUX_VALUE;
			}
			else if (((CURRENT_LATCHES.MICROINSTRUCTION[ADDR2MUX1] << 1) | CURRENT_LATCHES.MICROINSTRUCTION[ADDR2MUX0]) == 2) {
				ADDR2MUX_VALUE = CURRENT_LATCHES.IR & PCoffset9;
				ADDR2MUX_VALUE = (ADDR2MUX_VALUE & check_sign_PCoffset9) ? (ADDR2MUX_VALUE | sign_extend_PCoffset9) : ADDR2MUX_VALUE;
			}
			else if (((CURRENT_LATCHES.MICROINSTRUCTION[ADDR2MUX1] << 1) | CURRENT_LATCHES.MICROINSTRUCTION[ADDR2MUX0]) == 3) {
				ADDR2MUX_VALUE = CURRENT_LATCHES.IR & PCoffset11;
				ADDR2MUX_VALUE = (ADDR2MUX_VALUE & check_sign_PCoffset11) ? (ADDR2MUX_VALUE | sign_extend_PCoffset11) : ADDR2MUX_VALUE;

			}

			if (CURRENT_LATCHES.MICROINSTRUCTION[LSHF1]) {
				ADDR2MUX_VALUE = ADDR2MUX_VALUE << 1;
			}


			NEXT_LATCHES.PC = ADDR1MUX_VALUE + ADDR2MUX_VALUE;




		}



	}



}