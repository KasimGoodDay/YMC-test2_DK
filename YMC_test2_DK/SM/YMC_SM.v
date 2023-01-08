//******************************************************************************************************
//
//        Copyright (c) 2022 Yield Microelectronics Corp.
//                         All Rights Reserved.
//
//******************************************************************************************************
//
//	Model Name		:	YMG8K16F18L5BR1_Y10_PA_TOP
//	Creation Date	:	April 27, 2022
//	Description		:	8Kx16 bits Embedded MTP
//	Version			:	V10
//
//------------------------------------------------------------------------------------------------------
//	Ports
//		ADR[12:0]	:	Address input pin.
//		DIN[15:0]	:	Data input pin.
//		DOUT[15:0]	:	Data output pin.
//		CS			:	Chip selective input, active HIGH.
//		READ		:	Read enable input, active HIGH.
//		WR			:	Write enable input, active HIGH.
//		CLEN		:	Define test mode of measure memory cell current.
//		IFREN		:	IFREN = H to select the Information block.
//		EEPROM		:	EEPROM = H to select the EEPROM block.
//		SRL			:	Select default or shadow cell in MRGN READ / CLEN operation.
//		MRGN		:	MRGN = H to select MRGN READ / MRGN INTHV WRITE operation for data retention test when select the EEPROM block.
//		CLKIN		:	Define User-Supplied clock cycle under 12.5us (typ.). CLKIN must be stable before asserting WR.
//		ISAVB		:	Option setting for Read operation : ISAVB = H to select the high speed mode. ISAVB = L to select the low power or power saving mode.
//		STATICEN	:	STATICEN = H, STATIC mode (IDS <500uA). STATICEN = L, NON-STATIC mode (INDS <10uA).
//		BUSY		:	Flag indicating that write operation is complete or not.
//		VDD			:	Power supply
//		VSS			:	Ground
//------------------------------------------------------------------------------------------------------
//	Revision history:
//	Y10
//	2022/04/27	V10	:	Initial draft
//------------------------------------------------------------------------------------------------------
`define Tpwcss 1000
`define Tpwcsh 0
`define Tcsctrls 100
`define Tads 15
`define Tdoh 0
`define Tadhr 5
`define Tclk_min 10000
`define Tclk_max 15000
`define Twpw 100
`define Tbas 100
`define Twrc 1000
`define Tmcw 100
`define Tadhc 0
`define Twr_changed_MI 300000
`define Twr_changed_E 700000
`define Twr_nonchanged_MI 20000
`define Twr_nonchanged_E 80000
`define Trc_HMI 60
`define Trc_HE 300
`define Trac_HMI 60
`define Trac_HE 300
`define Trc_LMI 500
`define Trac_LMI 500
`define Trc_P 2000
`define Trac_P 2000
`timescale 1ns/1ps
`define numAddr 13
`define numOut 16
`define IFRENDepth 32
`define EEPROMDepth 128
`define Unknown 1'bx
`define Floating 1'bz
`define numOutx8 8
`define BUSY2READ 1000

`celldefine
module  YMG8K16F18L5BR1_Y10_PA_TOP
(
	ADR,
	DIN,
	DOUT,
	CS,
	READ,
	WR,
	CLEN,
	IFREN,
	EEPROM,
	SRL,
	MRGN,
	CLKIN,
	ISAVB,
	STATICEN,
	BUSY,
	VDD,
	VSS
);

input [`numAddr-1:0] ADR;
input [`numOut-1:0] DIN;
output [`numOut-1:0] DOUT;
input CS;
input READ;
input WR;
input CLEN;
input IFREN;
input EEPROM;
input SRL;
input MRGN;
input CLKIN;
input ISAVB;
input STATICEN;
output BUSY;
input VDD;
input VSS;

`protect

wire [`numAddr-1:0] ADR_buf;
wire [`numOut-1:0] DIN_buf;
wire [`numOut-1:0] DOUT_buf;
wire CS_buf;
wire READ_buf;
wire WR_buf;
wire CLEN_buf;
wire IFREN_buf;
wire EEPROM_buf;
wire SRL_buf;
wire MRGN_buf;
wire CLKIN_buf;
wire ISAVB_buf;
wire STATICEN_buf;
wire BUSY_buf;
wire VDD_buf;
wire VSS_buf;

buf adr[`numAddr-1:0](ADR_buf,ADR);
buf din[`numOut-1:0](DIN_buf,DIN);
buf dout[`numOut-1:0](DOUT,DOUT_buf);
buf (CS_buf,CS);
buf (READ_buf,READ);
buf (WR_buf,WR);
buf (CLEN_buf,CLEN);
buf (IFREN_buf,IFREN);
buf (EEPROM_buf,EEPROM);
buf (SRL_buf,SRL);
buf (MRGN_buf,MRGN);
buf (CLKIN_buf,CLKIN);
buf (ISAVB_buf,ISAVB);
buf (STATICEN_buf,STATICEN);
buf (BUSY,BUSY_buf);
buf (VDD_buf,VDD);
buf (VSS_buf,VSS);

wire
	MTP_BLOCK		= !IFREN & !EEPROM ,
	IFREN_BLOCK		=  IFREN & !EEPROM ,
	EEPROM_BLOCK	= !IFREN &  EEPROM ,
	ALL_BLOCK		= MTP_BLOCK | IFREN_BLOCK | EEPROM_BLOCK ,

	srlhorl			= (SRL!==`Floating),

	clkinhorl		= (CLKIN!==`Floating),
	isavbhorl		= (ISAVB!==`Floating),
	staticenhorl	= (STATICEN!==`Floating),

	High_speed		=  ISAVB &  STATICEN ,
	Low_power		= !ISAVB &  STATICEN ,
	Power_saving	= !ISAVB & !STATICEN ,

	NORMAL_READ		= !MRGN & !SRL          & (High_speed | Low_power | Power_saving) ,
	INTHV_WRITE		= !MRGN & !SRL          & isavbhorl & staticenhorl ,
	MRGN_READ		=  MRGN & srlhorl       & (High_speed | Low_power | Power_saving) ,
	MRGN_INTHV		=  MRGN & !SRL          & isavbhorl & staticenhorl ,

	READ_OPERATION			= ALL_BLOCK    & NORMAL_READ,
	INTHV_WRITE_OPERATION	= ALL_BLOCK    & INTHV_WRITE,
	MRGN_READ_OPERATION		= EEPROM_BLOCK & MRGN_READ,
	MRGN_INTHV_OPERATION	= EEPROM_BLOCK & MRGN_INTHV,

	dinhorl	=	( DIN[15]!==(`Floating) ) & ( DIN[14]!==(`Floating) ) & ( DIN[13]!==(`Floating) ) & ( DIN[12]!==(`Floating) ) &
				( DIN[11]!==(`Floating) ) & ( DIN[10]!==(`Floating) ) & ( DIN[9] !==(`Floating) ) & ( DIN[8] !==(`Floating) ) &
				( DIN[7] !==(`Floating) ) & ( DIN[6] !==(`Floating) ) & ( DIN[5] !==(`Floating) ) & ( DIN[4] !==(`Floating) ) &
				( DIN[3] !==(`Floating) ) & ( DIN[2] !==(`Floating) ) & ( DIN[1] !==(`Floating) ) & ( DIN[0] !==(`Floating) ) ,

	adrhorl	=	( ADR[12]!==(`Floating) ) &
				( ADR[11]!==(`Floating) ) & ( ADR[10]!==(`Floating) ) & ( ADR[9] !==(`Floating) ) & ( ADR[8] !==(`Floating) ) &
				( ADR[7] !==(`Floating) ) & ( ADR[6] !==(`Floating) ) & ( ADR[5] !==(`Floating) ) & ( ADR[4] !==(`Floating) ) &
				( ADR[3] !==(`Floating) ) & ( ADR[2] !==(`Floating) ) & ( ADR[1] !==(`Floating) ) & ( ADR[0] !==(`Floating) ) ,

	DIFF_READ	= READ_OPERATION | MRGN_READ_OPERATION     ,
	DIFF_WR		= INTHV_WRITE_OPERATION  | MRGN_INTHV_OPERATION  ,

	POWER		= !VSS & VDD & clkinhorl & adrhorl ,

	no_read		= POWER & CS         & !WR & !CLEN & DIFF_READ  & !BUSY ,
	no_write	= POWER & CS & !READ       & !CLEN & DIFF_WR    & dinhorl ,

	all_noread_mode		= no_read ,
	all_nowrite_mode	= no_write ,

	read			= no_read & READ ,
	write			= no_write & WR ,

	all_read_mode	= read ,
	all_write_mode	= write ,

	write_time	= all_write_mode | BUSY ,
	control_pins	= CS | READ | WR | CLEN ;

reg tpwcss,tpwcsh;
always @ (tpwcss) if($realtime > 0) MTP.DisplayError;
always @ (tpwcsh) if($realtime > 0) MTP.DisplayWarning;
specify
	$hold(posedge VDD,posedge CS,`Tpwcss,tpwcss);
	$hold(negedge CS,negedge VDD,`Tpwcsh,tpwcsh);
endspecify

reg tcsctrlsR,tadsr,tadhr;
always @ (tcsctrlsR,tadsr) if($realtime > 0) MTP.DisplayError;
always @ (tadhr) if($realtime > 0) MTP.DisplayWarning;
specify
	$hold(posedge CS,posedge READ,`Tcsctrls,tcsctrlsR);
	$hold(ADR,posedge READ,`Tads,tadsr);
	$hold(IFREN,posedge READ,`Tads,tadsr);
	$hold(EEPROM,posedge READ,`Tads,tadsr);
	$hold(negedge READ,ADR &&& no_read,`Tadhr,tadhr);
	$hold(negedge READ,IFREN &&& no_read,`Tadhr,tadhr);
	$hold(negedge READ,EEPROM &&& no_read,`Tadhr,tadhr);
endspecify

reg tcsctrlsW,twpw,tadsw,tbz2read,tclk,twrc,tnbc,tncpw,tnrpw;
always @ (tcsctrlsW,twpw,tadsw) if($realtime > 0) MTP.DisplayError;
always @ (tbz2read,tclk,twrc,tnbc,tncpw,tnrpw) if($realtime > 0) MTP.DisplayWarning;
specify
	$hold(posedge CS,posedge WR &&& all_write_mode,`Tcsctrls,tcsctrlsW);
	$width(posedge WR &&& all_nowrite_mode,`Twpw,0,twpw);
	$hold(ADR,posedge WR,`Tads,tadsw);
	$hold(DIN,posedge WR,`Tads,tadsw);
	$hold(IFREN,posedge WR,`Tads,tadsw);
	$hold(EEPROM,posedge WR,`Tads,tadsw);
	$width(posedge CLKIN &&& write_time,`Tclk_min/2,0,tclk);
	$width(negedge CLKIN &&& write_time,`Tclk_min/2,0,tclk);
	$recovery(negedge BUSY,posedge READ,`BUSY2READ,tbz2read);
	$recovery(negedge BUSY,posedge WR,`Twrc,twrc);
	$recovery(negedge BUSY,negedge CS,`Twrc,tnbc);
	$width(negedge CS,100,0,tncpw);
	$width(negedge READ &&& CS,100,0,tnrpw);
endspecify

Internal MTP(
	.ADR		(ADR_buf),
	.DIN		(DIN_buf),
	.DOUT		(DOUT_buf),
	.CS			(CS_buf),
	.READ		(READ_buf),
	.WR			(WR_buf),
	.CLEN		(CLEN_buf),
	.IFREN		(IFREN_buf),
	.EEPROM		(EEPROM_buf),
	.SRL		(SRL_buf),
	.MRGN		(MRGN_buf),
	.ISAVB		(ISAVB_buf),
	.STATICEN	(STATICEN_buf),
	.CLKIN		(CLKIN_buf),
	.BUSY		(BUSY_buf),
	.VDD		(VDD_buf),
	.VSS		(VSS_buf)
);

endmodule
`endcelldefine

`celldefine
module Internal
(
	ADR,
	DIN,
	DOUT,
	CS,
	READ,
	WR,
	CLEN,
	IFREN,
	EEPROM,
	SRL,
	MRGN,
	CLKIN,
	ISAVB,
	STATICEN,
	BUSY,
	VDD,
	VSS
);

input [`numAddr-1:0] ADR;
input [`numOut-1:0] DIN;
output [`numOut-1:0] DOUT;
input CS;
input READ;
input WR;
input CLEN;
input IFREN;
input EEPROM;
input SRL;
input MRGN;
input CLKIN;
input ISAVB;
input STATICEN;
output BUSY;
input VDD;
input VSS;

reg [`numOut-1:0] DOUT,data;
reg BUSY,DOUT_HOLD;
reg [`numAddr-1:0] ADR_LTCH,adrlech,ADR_CHG;
reg [`numOut-1:0] DIN_LTCH;
reg IFREN_LTCH,ifrenlech,IFREN_CHG;
reg EEPROM_LTCH,eepromlech,EEPROM_CHG;
reg SRL_LTCH,srllech,SRL_CHG;
reg isavblech,staticenlech;
parameter MTPDepth=2**`numAddr;
reg abort,WorkNG,clk_reg;
integer warning,c,clk_cou;
real readtime,adrtime;
reg readflag,firstread;
reg readerror,writeerror;

wire [`numAddr-1:0] MTP_addr = ADR_LTCH;
wire [`numAddr-1:0] IFREN_addr = ADR_LTCH;
wire [`numAddr-1:0] EEPROM_addr = ADR_LTCH;
reg [`numOut-1:0] MTP_mem [MTPDepth-1:0];
reg [`numOut-1:0] IFREN_mem [`IFRENDepth-1:0];
reg [`numOutx8-1:0] EEPROM_default_mem [`EEPROMDepth-1:0];
reg [`numOutx8-1:0] EEPROM_shadow_mem [`EEPROMDepth-1:0];
wire
	MTP_BLOCK		= !IFREN & !EEPROM ,
	IFREN_BLOCK		=  IFREN & !EEPROM ,
	EEPROM_BLOCK	= !IFREN &  EEPROM ;

initial begin
	DOUT_HOLD=0;
	BUSY=0;
	isavblech=`Unknown;
	staticenlech=`Unknown;
	firstread=0;
	readflag=0;
	clk_reg=0;
	clk_cou=0;
	readerror=0;
	writeerror=0;
	abort=0;
	WorkNG=1;
	for(c=0;c<MTPDepth;c=c+1) MTP_mem[c] = $random;
	for(c=0;c<`IFRENDepth;c=c+1) IFREN_mem[c] = $random;
	for(c=0;c<`EEPROMDepth;c=c+1)begin
		EEPROM_default_mem[c] = $random;
		EEPROM_shadow_mem[c] = $random;
	end
end

always @ (posedge DOUT_HOLD) begin
	#`Tdoh DOUT={`numOut{`Unknown}};
	DOUT_HOLD=0;
end

always @ (ISAVB) isavblech=`Unknown;
always @ (STATICEN) staticenlech=`Unknown;

always @ (*) begin
	adrtime=$realtime;
	if( YMG8K16F18L5BR1_Y10_PA_TOP.read & readflag )begin
		DOUT_HOLD=1;
		#0.01;
		ADR_CHG=ADR;
		IFREN_CHG=IFREN;
		EEPROM_CHG=EEPROM;
		SRL_CHG=SRL;
	end
end

always @ (ADR_CHG) begin
	#0.01;
	if(ADR_CHG===ADR)begin
		if( YMG8K16F18L5BR1_Y10_PA_TOP.read & readflag & !firstread ) begin
			if ( adrtime!=readtime ) begin
				YMG8K16F18L5BR1_Y10_PA_TOP.tcsctrlsR=`Unknown;
				YMG8K16F18L5BR1_Y10_PA_TOP.tadsr=`Unknown;
			end
			ifrenlech=IFREN;
			eepromlech=EEPROM;
			srllech=SRL;
			adrlech=ADR;
			ReadMem;
		end
	end
end

always @ (READ) begin
	if(READ)begin: readmode
		if(!CS&VDD) begin
			$display(" Time: %f ns, Warning! Please make sure this status is in STANDBY mode, not in read mode.", $realtime);
			readerror=1;
			disable readmode;
		end
		readflag=1;
		firstread=1;
		readtime=$realtime;
		DOUT_HOLD=1;
		#0.02;
		isavblech=ISAVB;
		staticenlech=STATICEN;
		adrlech=ADR;
		ifrenlech=IFREN;
		eepromlech=EEPROM;
		srllech=SRL;
		if(YMG8K16F18L5BR1_Y10_PA_TOP.all_read_mode) ReadMem;
		firstread=0;
	end
	else begin
		YMG8K16F18L5BR1_Y10_PA_TOP.tcsctrlsR=`Unknown;
		YMG8K16F18L5BR1_Y10_PA_TOP.tadsr=`Unknown;
		readflag=0;
		readerror=0;
	end
end

always @ (WR) begin
	if(WR) begin: writemode
		if(!CS&VDD) begin
			$display(" Time: %f ns, Warning! Please make sure this status is in STANDBY mode, not in write mode.", $realtime);
			writeerror=1;
			disable writemode;
		end
		#1;
		readflag=0;
		YMG8K16F18L5BR1_Y10_PA_TOP.twpw=`Unknown;
		WorkNG=0;
		ADR_LTCH=ADR;
		DIN_LTCH=DIN;
		clk_reg=CLKIN;
		IFREN_LTCH=IFREN;
		EEPROM_LTCH=EEPROM;
		if(YMG8K16F18L5BR1_Y10_PA_TOP.all_write_mode&(!BUSY)) WrMem;
	end
	else begin
		YMG8K16F18L5BR1_Y10_PA_TOP.tcsctrlsW=`Unknown;
		YMG8K16F18L5BR1_Y10_PA_TOP.tadsw=`Unknown;
		writeerror=0;
	end
end

always @ (posedge WR)begin
	if(BUSY) begin $display(" Time: %f ns, Error! Write operation is not completed when BUSY signal is high.", $realtime);#1 $stop;end
	else begin: busymode
		#1;
		if(YMG8K16F18L5BR1_Y10_PA_TOP.write&(!BUSY)&(!writeerror)) begin#(`Tbas-1) BUSY=1; DOUT={`numOut{`Unknown}};end
	end
end

always @ (posedge CS) begin
	ifrenlech=IFREN;
	eepromlech=EEPROM;
	srllech=SRL;
	adrlech=ADR;
	if (readerror) begin $display(" Time: %f ns, Error! CS Setup Time in Read cycle (Tcsctrls) = %f ns.", $realtime,`Tcsctrls); #1 $stop; end
	else if (writeerror) begin $display(" Time: %f ns, Error! CS Setup Time in Write cycle (Tcsctrls) = %f ns.", $realtime,`Tcsctrls); #1 $stop; end
end

always @ (negedge CS) begin
	if (READ) $display(" Time: %f ns, Warning! CS signal conflicted the Read Operation.", $realtime);
	else if (WR | BUSY) $display(" Time: %f ns, Warning! CS signal conflicted the Write Operation.", $realtime);
	BUSY=0; readflag=0;
end

always @ (posedge CLEN) if(!CS&VDD) $display(" Time: %f ns, Warning! Please make sure this status is in STANDBY mode, not in CLEN mode.", $realtime);

always @ (negedge VDD) begin
	if (CS) YMG8K16F18L5BR1_Y10_PA_TOP.tpwcsh=1;
	BUSY=0; readflag=0; DOUT={`numOut{`Unknown}};
end

wire All_Pinout = !CS & !READ & !WR & !CLEN & !VSS;

always @ (posedge VDD)
	if (!All_Pinout) begin
		$display(" Time: %f ns, Error! When VDD rises, all pins should remain logic 'L' state.", $realtime);
		#2 $stop;
	end

always @ (posedge YMG8K16F18L5BR1_Y10_PA_TOP.control_pins) begin
	if (!VDD) begin
		$display(" Time: %f ns, Error! VDD should rising up than Another Control Pins.", $realtime);
		#1 $stop;
	end
end

always @ (*) begin
	if(READ & WR) begin
		$display(" Time: %f ns, Error! READ and WR should not rising up at the same time.", $realtime);
		DOUT={`numOut{`Unknown}};
	end
	else if(READ & CLEN)begin
		$display(" Time: %f ns, Error! READ and CLEN should not rising up at the same time.", $realtime);
		DOUT={`numOut{`Unknown}};
	end
	else if(WR & CLEN) $display(" Time: %f ns, Error! CLEN and WR should not rising up at the same time.", $realtime);
end

always @ (WR & READ) if(WR & READ) $display(" Time: %f ns, Error! WR and READ should not turn on at the same time.", $realtime);

task DisplayError;
	begin
		case (YMG8K16F18L5BR1_Y10_PA_TOP.tpwcss)
			1'b0,1'b1: begin
				$display(" Time: %f ns, Error! VDD Setup Time before CS rises (Tpwcss) = %f ns.", $realtime,`Tpwcss);
				#2 $stop;
			end
			default: YMG8K16F18L5BR1_Y10_PA_TOP.tpwcss=`Unknown;
		endcase
		case (YMG8K16F18L5BR1_Y10_PA_TOP.tcsctrlsR)
			1'b0,1'b1: $display(" Time: %f ns, Error! CS Setup Time in Read cycle (Tcsctrls) = %f ns.", $realtime,`Tcsctrls);
			default: YMG8K16F18L5BR1_Y10_PA_TOP.tcsctrlsR=`Unknown;
		endcase
		case (YMG8K16F18L5BR1_Y10_PA_TOP.tcsctrlsW)
			1'b0,1'b1: $display(" Time: %f ns, Error! CS Setup Time in Write cycle (Tcsctrls) = %f ns.", $realtime,`Tcsctrls);
			default: YMG8K16F18L5BR1_Y10_PA_TOP.tcsctrlsW=`Unknown;
		endcase
		case (YMG8K16F18L5BR1_Y10_PA_TOP.tadsr)
			1'b0,1'b1: $display(" Time: %f ns, Error! ADR/IFREN/EEPROM Setup Time in Read cycle (Tads) = %f ns.", $realtime,`Tads);
			default: YMG8K16F18L5BR1_Y10_PA_TOP.tadsr=`Unknown;
		endcase
		case (YMG8K16F18L5BR1_Y10_PA_TOP.tadsw)
			1'b0,1'b1: $display(" Time: %f ns, Error! ADR/DIN/IFREN/EEPROM Setup Time in Write cycle (Tads) = %f ns.", $realtime,`Tads);
			default: YMG8K16F18L5BR1_Y10_PA_TOP.tadsw=`Unknown;
		endcase
		case (YMG8K16F18L5BR1_Y10_PA_TOP.twpw)
			1'b0,1'b1: $display(" Time: %f ns, Error! WR Pulse Width Min(Twpw) = %f ns.", $realtime,`Twpw);
			default: YMG8K16F18L5BR1_Y10_PA_TOP.twpw=`Unknown;
		endcase
	end
endtask

task DisplayWarning;
	begin
		case (YMG8K16F18L5BR1_Y10_PA_TOP.tpwcsh)
			1'b0,1'b1: begin
				$display(" Time: %f ns, Warning! VDD Hold Time after CS falls (Tpwcsh) = %f ns.", $realtime,`Tpwcsh);
				#1 YMG8K16F18L5BR1_Y10_PA_TOP.tpwcsh=`Unknown;
			end
		endcase
		case (YMG8K16F18L5BR1_Y10_PA_TOP.tadhr)
			1'b0,1'b1: begin
				$display(" Time: %f ns, Warning! ADR/IFREN/EEPROM Hold Time for Read (Tadhr) = %f ns.", $realtime,`Tadhr);
				#1 YMG8K16F18L5BR1_Y10_PA_TOP.tadhr=`Unknown;
				DOUT={`numOut{`Unknown}};
			end
		endcase
		case (YMG8K16F18L5BR1_Y10_PA_TOP.twrc)
			1'b0,1'b1: begin
				$display(" Time: %f ns, Warning! Write Recovery Time (Twrc) = %f ns.", $realtime,`Twrc);
				#1 YMG8K16F18L5BR1_Y10_PA_TOP.twrc=`Unknown;
			end
		endcase
		case (YMG8K16F18L5BR1_Y10_PA_TOP.tnbc)
			1'b0,1'b1: begin
				$display(" Time: %f ns, Warning! CS Hold Time after BUSY falls (Twrc) = %f ns.", $realtime,`Twrc);
				#1 YMG8K16F18L5BR1_Y10_PA_TOP.tnbc=`Unknown;
			end
		endcase
		case (YMG8K16F18L5BR1_Y10_PA_TOP.tclk)
			1'b0,1'b1: begin
				$display(" Time: %f ns, Warning! Min. Clock Cycle Time (Tclk_min) = %f ns.", $realtime,`Tclk_min);
				#1 YMG8K16F18L5BR1_Y10_PA_TOP.tclk=`Unknown;
			end
		endcase
		case (YMG8K16F18L5BR1_Y10_PA_TOP.tbz2read)
			1'b0,1'b1: begin
				$display(" Time: %f ns, Warning! Data access operation is allowable in 1 us later after BUSY is pulled to L. Otherwise incorrect Data may be read-out.", $realtime);
				#1 YMG8K16F18L5BR1_Y10_PA_TOP.tbz2read=`Unknown;
			end
		endcase
		case (YMG8K16F18L5BR1_Y10_PA_TOP.tncpw)
			1'b0,1'b1: begin
				$display(" Time: %f ns, Warning! The next CS signal can be pulled 'H' after 100 ns.", $realtime);
				#1 YMG8K16F18L5BR1_Y10_PA_TOP.tncpw=`Unknown;
			end
		endcase
		case (YMG8K16F18L5BR1_Y10_PA_TOP.tnrpw)
			1'b0,1'b1: begin
				$display(" Time: %f ns, Warning! The next READ signal can be pulled 'H' after 100 ns.", $realtime);
				#1 YMG8K16F18L5BR1_Y10_PA_TOP.tnrpw=`Unknown;
			end
		endcase
	end
endtask

task ReadMem;
	integer i,truthtable_wrong,trac;
	begin: readmemory
		ADR_LTCH=ADR;
		IFREN_LTCH=IFREN;
		EEPROM_LTCH=EEPROM;
		SRL_LTCH=SRL;
		truthtable_wrong=0;
		abort=0;
		trac=0;
		i=0;
		begin: loop_read_pulse
			if( YMG8K16F18L5BR1_Y10_PA_TOP.High_speed & (MTP_BLOCK | IFREN_BLOCK) ) trac=`Trac_HMI;
			else if( YMG8K16F18L5BR1_Y10_PA_TOP.High_speed & EEPROM_BLOCK ) trac=`Trac_HE;
			else if( YMG8K16F18L5BR1_Y10_PA_TOP.Low_power & (MTP_BLOCK | IFREN_BLOCK) ) trac=`Trac_LMI;
			else if( YMG8K16F18L5BR1_Y10_PA_TOP.Power_saving ) trac=`Trac_P;
			while(i<((trac-0.02)*100)) begin
				if((VSS)|(!VDD)|(!CS)|(!READ)) begin
					abort=1;
					disable loop_read_pulse;
				end
				else if(~&YMG8K16F18L5BR1_Y10_PA_TOP.all_noread_mode)begin
					if (!truthtable_wrong) $display(" Time: %f ns, ERROR! Please refer to the spec of the truth table.( Read Mode ).", $realtime-1);
					truthtable_wrong=1;
					disable readmemory;
				end
				else if((adrlech!==ADR) & YMG8K16F18L5BR1_Y10_PA_TOP.read) disable readmemory;
				else if((ifrenlech!==IFREN) & YMG8K16F18L5BR1_Y10_PA_TOP.read) disable readmemory;
				else if((eepromlech!==EEPROM) & YMG8K16F18L5BR1_Y10_PA_TOP.read) disable readmemory;
				else if((srllech!==SRL) & YMG8K16F18L5BR1_Y10_PA_TOP.read) disable readmemory;
				else if(isavblech!==ISAVB) begin
					$display(" Time: %f ns, Error! To change the option setting for read operation, the READ signal must be low.", $realtime-1);
					disable readmemory;
				end
				else if(staticenlech!==STATICEN) begin
					$display(" Time: %f ns, Error! To change the option setting for read operation, the READ signal must be low.", $realtime-1);
					disable readmemory;
				end
				else if(!YMG8K16F18L5BR1_Y10_PA_TOP.tcsctrlsR | YMG8K16F18L5BR1_Y10_PA_TOP.tcsctrlsR |
					!YMG8K16F18L5BR1_Y10_PA_TOP.tadsr | YMG8K16F18L5BR1_Y10_PA_TOP.tadsr ) begin
					abort=1;
					disable loop_read_pulse;
				end
				else begin
					#0.01;
					i=i+1;
				end
			end
		end
		ADR_CHG=`Unknown;
		IFREN_CHG=`Unknown;
		EEPROM_CHG=`Unknown;
		SRL_CHG=`Unknown;
		if(abort) abort=0;
		else if ( YMG8K16F18L5BR1_Y10_PA_TOP.High_speed ) begin
			if (MTP_BLOCK) DOUT=MTP_mem[MTP_addr];
			else if (IFREN_BLOCK) DOUT=IFREN_mem[IFREN_addr];
			else if (EEPROM_BLOCK) begin
				if( !SRL ) DOUT={{`numOut{`Unknown}},EEPROM_default_mem[EEPROM_addr]};
				else if( MRGN & SRL ) DOUT={{`numOut{`Unknown}},EEPROM_shadow_mem[EEPROM_addr]};
			end
		end
		else if ( YMG8K16F18L5BR1_Y10_PA_TOP.Low_power ) begin
			if (MTP_BLOCK) DOUT=MTP_mem[MTP_addr];
			else if (IFREN_BLOCK) DOUT=IFREN_mem[IFREN_addr];
		end
		else if ( YMG8K16F18L5BR1_Y10_PA_TOP.Power_saving ) begin
			if (MTP_BLOCK) DOUT=MTP_mem[MTP_addr];
			else if (IFREN_BLOCK) DOUT=IFREN_mem[IFREN_addr];
			else if (EEPROM_BLOCK) begin
				if( !SRL ) DOUT={{`numOut{`Unknown}},EEPROM_default_mem[EEPROM_addr]};
				else if( MRGN & SRL ) DOUT={{`numOut{`Unknown}},EEPROM_shadow_mem[EEPROM_addr]};
			end
		end
	end
endtask

task WrMem;
	integer z,z_mi,z_m,z_e,i,ii,thvwrtime,truthtable_wrong;
	begin
		z_mi=(`Twr_changed_MI*1.2)-(`Twr_changed_MI*0.8)+1;
		z_e=(`Twr_changed_E*1.2)-(`Twr_changed_E*0.8)+1;
		i=0;
		ii=0;
		abort=0;
		clk_cou=0;
		thvwrtime=0;
		truthtable_wrong=0;
		warning=0;
		begin:loop_int_wrt
			if( (MTP_mem[ADR_LTCH]===DIN_LTCH) & MTP_BLOCK ) thvwrtime=`Twr_nonchanged_MI;
			else if( (IFREN_mem[ADR_LTCH]===DIN_LTCH) & IFREN_BLOCK ) thvwrtime=`Twr_nonchanged_MI;
			else if( (EEPROM_default_mem[ADR_LTCH]===DIN_LTCH[`numOutx8-1:0]) & EEPROM_BLOCK ) thvwrtime=`Twr_nonchanged_E;
			else if(!EEPROM) thvwrtime=(`Twr_changed_MI*0.8)+({$random}%z_mi);
			else thvwrtime=(`Twr_changed_E*0.8)+({$random}%z_e);
			while(i<(thvwrtime-2)) begin
				if((VSS)|(!VDD)|(!CS)) begin
					abort=1;
					WorkNG=1;
					disable loop_int_wrt;
				end
				else if((ADR_LTCH!==ADR) & (ii===0)) begin
					$display(" Time: %f ns, Error! ADR signal conflicted the Write Cycle.", $realtime-1);
					abort=1;
					WorkNG=1;
					ii=1;
				end
				else if(DIN_LTCH!==DIN & (ii===0)) begin
					$display(" Time: %f ns, Error! DIN signal conflicted the Write Cycle.", $realtime-1);
					abort=1;
					WorkNG=1;
					ii=1;
				end
				else if(IFREN_LTCH!==IFREN & (ii===0) ) begin
					$display(" Time: %f ns, Error! IFREN signal conflicted the Write Cycle.", $realtime-1);
					abort=1;
					WorkNG=1;
					ii=1;
				end
				else if(EEPROM_LTCH!==EEPROM & (ii===0) ) begin
					$display(" Time: %f ns, Error! EEPROM signal conflicted the Write Cycle.", $realtime-1);
					abort=1;
					WorkNG=1;
					ii=1;
				end
				else if(~&YMG8K16F18L5BR1_Y10_PA_TOP.no_write)begin
					if (!truthtable_wrong) $display(" Time: %f ns, Error! Please refer to the spec of the truth table.( Write Mode ).", $realtime-1);
					truthtable_wrong=1;
					#1;
					i=i+1;
				end
				else if(!YMG8K16F18L5BR1_Y10_PA_TOP.tcsctrlsW | YMG8K16F18L5BR1_Y10_PA_TOP.tcsctrlsW |
						!YMG8K16F18L5BR1_Y10_PA_TOP.twpw | YMG8K16F18L5BR1_Y10_PA_TOP.twpw |
						!YMG8K16F18L5BR1_Y10_PA_TOP.tadsw | YMG8K16F18L5BR1_Y10_PA_TOP.tadsw ) begin
					YMG8K16F18L5BR1_Y10_PA_TOP.tcsctrlsW=`Unknown;
					YMG8K16F18L5BR1_Y10_PA_TOP.tadsw=`Unknown;
					YMG8K16F18L5BR1_Y10_PA_TOP.twpw=`Unknown;
					WorkNG=0;
					abort=1;
					ii=1;
				end
				else if(CLKIN===clk_reg)begin
					clk_cou=clk_cou+1;
					if (clk_cou==(`Tclk_max/2+1)&(thvwrtime==`Twr_changed_MI)) $display(" Time: %f ns, Warning! Max. CLKIN duty cycle (50/50) = %f ns.", $realtime,(`Tclk_max/2));
					if (warning===10) $stop;
					else if (clk_cou>`Tclk_max) begin
						$display(" Time: %f ns, Warning! CLKIN should be toggled in write mode.", $realtime-1);
						warning=warning+1;
						clk_cou=0;
					end
					#1;
					i=i+1;
				end
				else if(CLKIN==!clk_reg)begin
					clk_cou=0;
					clk_reg=CLKIN;
					#1;i=i+1;
				end
			end
		end
		#1;
		BUSY=0;
		truthtable_wrong=0;
		if(abort & (ii===0)) abort=0;
		else if (MTP_BLOCK & (ii===0)) MTP_mem[MTP_addr]=DIN_LTCH;
		else if (IFREN_BLOCK & (ii===0)) IFREN_mem[IFREN_addr]=DIN_LTCH;
		else if (EEPROM_BLOCK & !MRGN & (ii===0)) begin
			EEPROM_default_mem[EEPROM_addr]=DIN_LTCH[`numOutx8-1:0];
			EEPROM_shadow_mem[EEPROM_addr]=~DIN_LTCH[`numOutx8-1:0];
		end
		else if (EEPROM_BLOCK &  MRGN & (ii===0)) begin
			EEPROM_default_mem[EEPROM_addr]=DIN_LTCH[`numOutx8-1:0];
			EEPROM_shadow_mem[EEPROM_addr]=DIN_LTCH[`numOutx8-1:0];
		end
		else if (MTP_BLOCK) begin
			MTP_mem[ADR_LTCH]={`numOut{`Unknown}};
			MTP_mem[ADR]={`numOut{`Unknown}};
		end
		else if (IFREN_BLOCK) begin
			IFREN_mem[ADR_LTCH]={`numOut{`Unknown}};
			IFREN_mem[ADR]={`numOut{`Unknown}};
		end
		else if (EEPROM_BLOCK) begin
			EEPROM_default_mem[ADR_LTCH]={`numOut{`Unknown}};
			EEPROM_default_mem[ADR]={`numOut{`Unknown}};
			EEPROM_shadow_mem[ADR_LTCH]={`numOut{`Unknown}};
			EEPROM_shadow_mem[ADR]={`numOut{`Unknown}};
		end
	end
endtask

`endprotect
endmodule
`endcelldefine