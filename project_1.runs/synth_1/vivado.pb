
�
Command: %s
1870*	planAhead2�
�read_checkpoint -auto_incremental -incremental /home/sj/Documents/study/asic/project_1/project_1.srcs/utils_1/imports/synth_1/tb.dcpZ12-2866h px� 
�
;Read reference checkpoint from %s for incremental synthesis3154*	planAhead2W
U/home/sj/Documents/study/asic/project_1/project_1.srcs/utils_1/imports/synth_1/tb.dcpZ12-5825h px� 
T
-Please ensure there are no constraint changes3725*	planAheadZ12-7989h px� 
^
Command: %s
53*	vivadotcl2-
+synth_design -top TOP -part xc7k70tfbv676-1Z4-113h px� 
:
Starting synth_design
149*	vivadotclZ4-321h px� 
z
@Attempting to get a license for feature '%s' and/or device '%s'
308*common2
	Synthesis2	
xc7k70tZ17-347h px� 
j
0Got license for feature '%s' and/or device '%s'
310*common2
	Synthesis2	
xc7k70tZ17-349h px� 

VNo compile time benefit to using incremental synthesis; A full resynthesis will be run2353*designutilsZ20-5440h px� 
�
�Flow is switching to default flow due to incremental criteria not met. If you would like to alter this behaviour and have the flow terminate instead, please set the following parameter config_implementation {autoIncr.Synth.RejectBehavior Terminate}2229*designutilsZ20-4379h px� 
o
HMultithreading enabled for synth_design using a maximum of %s processes.4828*oasys2
7Z8-7079h px� 
a
?Launching helper process for spawning children vivado processes4827*oasysZ8-7078h px� 
N
#Helper process launched with PID %s4824*oasys2
54537Z8-7075h px� 
�
%s*synth2�
�Starting Synthesize : Time (s): cpu = 00:00:04 ; elapsed = 00:00:04 . Memory (MB): peak = 1751.402 ; gain = 267.855 ; free physical = 1836 ; free virtual = 12815
h px� 
�
synthesizing module '%s'%s4497*oasys2
TOP2
 2O
K/home/sj/Documents/study/asic/project_1/project_1.srcs/sources_1/new/top.sv2
38@Z8-6157h px� 
�
synthesizing module '%s'%s4497*oasys2
PG2
 2O
K/home/sj/Documents/study/asic/project_1/project_1.srcs/sources_1/new/top.sv2
368@Z8-6157h px� 
�
'done synthesizing module '%s'%s (%s#%s)4495*oasys2
PG2
 2
02
12O
K/home/sj/Documents/study/asic/project_1/project_1.srcs/sources_1/new/top.sv2
368@Z8-6155h px� 
�
synthesizing module '%s'%s4497*oasys2
Loadable_Counter2
 2O
K/home/sj/Documents/study/asic/project_1/project_1.srcs/sources_1/new/top.sv2
458@Z8-6157h px� 
�
'done synthesizing module '%s'%s (%s#%s)4495*oasys2
Loadable_Counter2
 2
02
12O
K/home/sj/Documents/study/asic/project_1/project_1.srcs/sources_1/new/top.sv2
458@Z8-6155h px� 
�
synthesizing module '%s'%s4497*oasys2
ROM2
 2O
K/home/sj/Documents/study/asic/project_1/project_1.srcs/sources_1/new/top.sv2
1028@Z8-6157h px� 
�
synthesizing module '%s'%s4497*oasys2
BRAM2
 2O
K/home/sj/Documents/study/asic/project_1/project_1.srcs/sources_1/new/top.sv2
728@Z8-6157h px� 
�
%s, ignoring3604*oasys2y
wcould not open $readmem data file 'mem_init.mem'; please make sure the file is added to project and has read permission2O
K/home/sj/Documents/study/asic/project_1/project_1.srcs/sources_1/new/top.sv2
858@Z8-4445h px� 
�
'done synthesizing module '%s'%s (%s#%s)4495*oasys2
BRAM2
 2
02
12O
K/home/sj/Documents/study/asic/project_1/project_1.srcs/sources_1/new/top.sv2
728@Z8-6155h px� 
�
Pwidth (%s) of port connection '%s' does not match port width (%s) of module '%s'689*oasys2
352
read_address2
42
BRAM2O
K/home/sj/Documents/study/asic/project_1/project_1.srcs/sources_1/new/top.sv2
1198@Z8-689h px� 
�
'done synthesizing module '%s'%s (%s#%s)4495*oasys2
ROM2
 2
02
12O
K/home/sj/Documents/study/asic/project_1/project_1.srcs/sources_1/new/top.sv2
1028@Z8-6155h px� 
�
synthesizing module '%s'%s4497*oasys2
S2P2
 2O
K/home/sj/Documents/study/asic/project_1/project_1.srcs/sources_1/new/top.sv2
1278@Z8-6157h px� 
�
'done synthesizing module '%s'%s (%s#%s)4495*oasys2
S2P2
 2
02
12O
K/home/sj/Documents/study/asic/project_1/project_1.srcs/sources_1/new/top.sv2
1278@Z8-6155h px� 
�
synthesizing module '%s'%s4497*oasys2
fft_processor2
 2O
K/home/sj/Documents/study/asic/project_1/project_1.srcs/sources_1/new/top.sv2
2258@Z8-6157h px� 
�
synthesizing module '%s'%s4497*oasys2
BF12
 2O
K/home/sj/Documents/study/asic/project_1/project_1.srcs/sources_1/new/top.sv2
1608@Z8-6157h px� 
�
'done synthesizing module '%s'%s (%s#%s)4495*oasys2
BF12
 2
02
12O
K/home/sj/Documents/study/asic/project_1/project_1.srcs/sources_1/new/top.sv2
1608@Z8-6155h px� 
�
synthesizing module '%s'%s4497*oasys2
BF22
 2O
K/home/sj/Documents/study/asic/project_1/project_1.srcs/sources_1/new/top.sv2
1688@Z8-6157h px� 
�
'done synthesizing module '%s'%s (%s#%s)4495*oasys2
BF22
 2
02
12O
K/home/sj/Documents/study/asic/project_1/project_1.srcs/sources_1/new/top.sv2
1688@Z8-6155h px� 
�
synthesizing module '%s'%s4497*oasys2
delay2
 2O
K/home/sj/Documents/study/asic/project_1/project_1.srcs/sources_1/new/top.sv2
2178@Z8-6157h px� 
�
'done synthesizing module '%s'%s (%s#%s)4495*oasys2
delay2
 2
02
12O
K/home/sj/Documents/study/asic/project_1/project_1.srcs/sources_1/new/top.sv2
2178@Z8-6155h px� 
�
synthesizing module '%s'%s4497*oasys2
CMX12
 2O
K/home/sj/Documents/study/asic/project_1/project_1.srcs/sources_1/new/top.sv2
1778@Z8-6157h px� 
�
'done synthesizing module '%s'%s (%s#%s)4495*oasys2
CMX12
 2
02
12O
K/home/sj/Documents/study/asic/project_1/project_1.srcs/sources_1/new/top.sv2
1778@Z8-6155h px� 
�
synthesizing module '%s'%s4497*oasys2
CMX22
 2O
K/home/sj/Documents/study/asic/project_1/project_1.srcs/sources_1/new/top.sv2
1978@Z8-6157h px� 
�
'done synthesizing module '%s'%s (%s#%s)4495*oasys2
CMX22
 2
02
12O
K/home/sj/Documents/study/asic/project_1/project_1.srcs/sources_1/new/top.sv2
1978@Z8-6155h px� 
�
'done synthesizing module '%s'%s (%s#%s)4495*oasys2
fft_processor2
 2
02
12O
K/home/sj/Documents/study/asic/project_1/project_1.srcs/sources_1/new/top.sv2
2258@Z8-6155h px� 
�
synthesizing module '%s'%s4497*oasys2
P2S2
 2O
K/home/sj/Documents/study/asic/project_1/project_1.srcs/sources_1/new/top.sv2
2948@Z8-6157h px� 
�
'done synthesizing module '%s'%s (%s#%s)4495*oasys2
P2S2
 2
02
12O
K/home/sj/Documents/study/asic/project_1/project_1.srcs/sources_1/new/top.sv2
2948@Z8-6155h px� 
�
'done synthesizing module '%s'%s (%s#%s)4495*oasys2
TOP2
 2
02
12O
K/home/sj/Documents/study/asic/project_1/project_1.srcs/sources_1/new/top.sv2
38@Z8-6155h px� 
�
+Unused sequential element %s was removed. 
4326*oasys2

out3_reg2O
K/home/sj/Documents/study/asic/project_1/project_1.srcs/sources_1/new/top.sv2
1728@Z8-6014h px� 
�
+Unused sequential element %s was removed. 
4326*oasys2
en_internal_reg2O
K/home/sj/Documents/study/asic/project_1/project_1.srcs/sources_1/new/top.sv2
2768@Z8-6014h px� 
�
+Unused sequential element %s was removed. 
4326*oasys2
counter_reg2O
K/home/sj/Documents/study/asic/project_1/project_1.srcs/sources_1/new/top.sv2
2778@Z8-6014h px� 
�
+Unused sequential element %s was removed. 
4326*oasys2
img_out_reg[0]2O
K/home/sj/Documents/study/asic/project_1/project_1.srcs/sources_1/new/top.sv2
2808@Z8-6014h px� 
u
9Port %s in module %s is either unconnected or has no load4866*oasys2
start2
fft_processorZ8-7129h px� 
i
9Port %s in module %s is either unconnected or has no load4866*oasys2
ren2
ROMZ8-7129h px� 
�
%s*synth2�
�Finished Synthesize : Time (s): cpu = 00:00:05 ; elapsed = 00:00:06 . Memory (MB): peak = 1825.371 ; gain = 341.824 ; free physical = 1747 ; free virtual = 12728
h px� 
l
%s
*synth2T
R---------------------------------------------------------------------------------
h p
x
� 
�
%s*synth2�
�Finished Constraint Validation : Time (s): cpu = 00:00:06 ; elapsed = 00:00:06 . Memory (MB): peak = 1843.184 ; gain = 359.637 ; free physical = 1747 ; free virtual = 12728
h px� 
l
%s
*synth2T
R---------------------------------------------------------------------------------
h p
x
� 
l
%s
*synth2T
R---------------------------------------------------------------------------------
h p
x
� 
D
%s
*synth2,
*Start Loading Part and Timing Information
h p
x
� 
l
%s
*synth2T
R---------------------------------------------------------------------------------
h p
x
� 
8
%s
*synth2 
Loading part: xc7k70tfbv676-1
h p
x
� 
D
Loading part %s157*device2
xc7k70tfbv676-1Z21-403h px� 
l
%s
*synth2T
R---------------------------------------------------------------------------------
h p
x
� 
�
%s*synth2�
�Finished Loading Part and Timing Information : Time (s): cpu = 00:00:06 ; elapsed = 00:00:06 . Memory (MB): peak = 1851.188 ; gain = 367.641 ; free physical = 1755 ; free virtual = 12736
h px� 
l
%s
*synth2T
R---------------------------------------------------------------------------------
h p
x
� 
l
%s
*synth2T
R---------------------------------------------------------------------------------
h p
x
� 
�
%s*synth2�
�Finished RTL Optimization Phase 2 : Time (s): cpu = 00:00:07 ; elapsed = 00:00:07 . Memory (MB): peak = 1860.094 ; gain = 376.547 ; free physical = 1741 ; free virtual = 12722
h px� 
l
%s
*synth2T
R---------------------------------------------------------------------------------
h p
x
� 
l
%s
*synth2T
R---------------------------------------------------------------------------------
h p
x
� 
:
%s
*synth2"
 Start RTL Component Statistics 
h p
x
� 
l
%s
*synth2T
R---------------------------------------------------------------------------------
h p
x
� 
9
%s
*synth2!
Detailed RTL Component Info : 
h p
x
� 
(
%s
*synth2
+---Adders : 
h p
x
� 
F
%s
*synth2.
,	   2 Input   16 Bit       Adders := 13    
h p
x
� 
F
%s
*synth2.
,	   3 Input   16 Bit       Adders := 13    
h p
x
� 
F
%s
*synth2.
,	   2 Input    3 Bit       Adders := 6     
h p
x
� 
+
%s
*synth2
+---Registers : 
h p
x
� 
H
%s
*synth20
.	               16 Bit    Registers := 85    
h p
x
� 
H
%s
*synth20
.	                3 Bit    Registers := 6     
h p
x
� 
H
%s
*synth20
.	                1 Bit    Registers := 13    
h p
x
� 
'
%s
*synth2
+---Muxes : 
h p
x
� 
F
%s
*synth2.
,	   2 Input   16 Bit        Muxes := 2     
h p
x
� 
F
%s
*synth2.
,	   2 Input    3 Bit        Muxes := 3     
h p
x
� 
F
%s
*synth2.
,	   2 Input    1 Bit        Muxes := 25    
h p
x
� 
l
%s
*synth2T
R---------------------------------------------------------------------------------
h p
x
� 
=
%s
*synth2%
#Finished RTL Component Statistics 
h p
x
� 
l
%s
*synth2T
R---------------------------------------------------------------------------------
h p
x
� 
l
%s
*synth2T
R---------------------------------------------------------------------------------
h p
x
� 
6
%s
*synth2
Start Part Resource Summary
h p
x
� 
l
%s
*synth2T
R---------------------------------------------------------------------------------
h p
x
� 
q
%s
*synth2Y
WPart Resources:
DSPs: 240 (col length:80)
BRAMs: 270 (col length: RAMB18 80 RAMB36 40)
h p
x
� 
l
%s
*synth2T
R---------------------------------------------------------------------------------
h p
x
� 
9
%s
*synth2!
Finished Part Resource Summary
h p
x
� 
l
%s
*synth2T
R---------------------------------------------------------------------------------
h p
x
� 
l
%s
*synth2T
R---------------------------------------------------------------------------------
h p
x
� 
E
%s
*synth2-
+Start Cross Boundary and Area Optimization
h p
x
� 
l
%s
*synth2T
R---------------------------------------------------------------------------------
h p
x
� 
H
&Parallel synthesis criteria is not met4829*oasysZ8-7080h px� 
i
%s
*synth2Q
ODSP Report: Generating DSP fftp/cmx2/mult_diff, operation Mode is: A*(B:0xb5).
h p
x
� 
m
%s
*synth2U
SDSP Report: operator fftp/cmx2/mult_diff is absorbed into DSP fftp/cmx2/mult_diff.
h p
x
� 
h
%s
*synth2P
NDSP Report: Generating DSP fftp/cmx1/mult_sum, operation Mode is: A*(B:0xb5).
h p
x
� 
k
%s
*synth2S
QDSP Report: operator fftp/cmx1/mult_sum is absorbed into DSP fftp/cmx1/mult_sum.
h p
x
� 
h
%s
*synth2P
NDSP Report: Generating DSP fftp/cmx2/mult_sum, operation Mode is: A*(B:0xb5).
h p
x
� 
k
%s
*synth2S
QDSP Report: operator fftp/cmx2/mult_sum is absorbed into DSP fftp/cmx2/mult_sum.
h p
x
� 
i
%s
*synth2Q
ODSP Report: Generating DSP fftp/cmx1/mult_diff, operation Mode is: A*(B:0xb5).
h p
x
� 
m
%s
*synth2U
SDSP Report: operator fftp/cmx1/mult_diff is absorbed into DSP fftp/cmx1/mult_diff.
h p
x
� 
l
%s
*synth2T
R---------------------------------------------------------------------------------
h p
x
� 
�
%s*synth2�
�Finished Cross Boundary and Area Optimization : Time (s): cpu = 00:00:11 ; elapsed = 00:00:11 . Memory (MB): peak = 1969.992 ; gain = 486.445 ; free physical = 1718 ; free virtual = 12699
h px� 
l
%s
*synth2T
R---------------------------------------------------------------------------------
h p
x
� 
_
%s
*synth2G
E Sort Area is  fftp/cmx1/mult_diff_3 : 0 0 : 471 471 : Used 1 time 0
h p
x
� 
^
%s
*synth2F
D Sort Area is  fftp/cmx1/mult_sum_0 : 0 0 : 471 471 : Used 1 time 0
h p
x
� 
_
%s
*synth2G
E Sort Area is  fftp/cmx2/mult_diff_2 : 0 0 : 471 471 : Used 1 time 0
h p
x
� 
^
%s
*synth2F
D Sort Area is  fftp/cmx2/mult_sum_4 : 0 0 : 471 471 : Used 1 time 0
h p
x
� 
�
%s*synth2�
�---------------------------------------------------------------------------------
Start ROM, RAM, DSP, Shift Register and Retiming Reporting
h px� 
l
%s*synth2T
R---------------------------------------------------------------------------------
h px� 
v
%s*synth2^
\
DSP: Preliminary Mapping Report (see note below. The ' indicates corresponding REG is set)
h px� 
�
%s*synth2~
|+------------+-------------+--------+--------+--------+--------+--------+------+------+------+------+-------+------+------+
h px� 
�
%s*synth2
}|Module Name | DSP Mapping | A Size | B Size | C Size | D Size | P Size | AREG | BREG | CREG | DREG | ADREG | MREG | PREG | 
h px� 
�
%s*synth2~
|+------------+-------------+--------+--------+--------+--------+--------+------+------+------+------+-------+------+------+
h px� 
�
%s*synth2
}|CMX2        | A*(B:0xb5)  | 16     | 9      | -      | -      | 25     | 0    | 0    | -    | -    | -     | 0    | 0    | 
h px� 
�
%s*synth2
}|CMX1        | A*(B:0xb5)  | 16     | 9      | -      | -      | 25     | 0    | 0    | -    | -    | -     | 0    | 0    | 
h px� 
�
%s*synth2
}|CMX2        | A*(B:0xb5)  | 16     | 9      | -      | -      | 25     | 0    | 0    | -    | -    | -     | 0    | 0    | 
h px� 
�
%s*synth2
}|CMX1        | A*(B:0xb5)  | 16     | 9      | -      | -      | 25     | 0    | 0    | -    | -    | -     | 0    | 0    | 
h px� 
�
%s*synth2
}+------------+-------------+--------+--------+--------+--------+--------+------+------+------+------+-------+------+------+

h px� 
�
%s*synth2�
�Note: The table above is a preliminary report that shows the DSPs inferred at the current stage of the synthesis flow. Some DSP may be reimplemented as non DSP primitives later in the synthesis flow. Multiple instantiated DSPs are reported only once.
h px� 
�
%s*synth2�
�---------------------------------------------------------------------------------
Finished ROM, RAM, DSP, Shift Register and Retiming Reporting
h px� 
l
%s*synth2T
R---------------------------------------------------------------------------------
h px� 
l
%s
*synth2T
R---------------------------------------------------------------------------------
h p
x
� 
4
%s
*synth2
Start Timing Optimization
h p
x
� 
l
%s
*synth2T
R---------------------------------------------------------------------------------
h p
x
� 
l
%s
*synth2T
R---------------------------------------------------------------------------------
h p
x
� 
�
%s*synth2�
�Finished Timing Optimization : Time (s): cpu = 00:00:11 ; elapsed = 00:00:11 . Memory (MB): peak = 1972.961 ; gain = 489.414 ; free physical = 1718 ; free virtual = 12699
h px� 
l
%s
*synth2T
R---------------------------------------------------------------------------------
h p
x
� 
l
%s
*synth2T
R---------------------------------------------------------------------------------
h p
x
� 
3
%s
*synth2
Start Technology Mapping
h p
x
� 
l
%s
*synth2T
R---------------------------------------------------------------------------------
h p
x
� 
l
%s
*synth2T
R---------------------------------------------------------------------------------
h p
x
� 
�
%s*synth2�
�Finished Technology Mapping : Time (s): cpu = 00:00:11 ; elapsed = 00:00:12 . Memory (MB): peak = 1972.961 ; gain = 489.414 ; free physical = 1719 ; free virtual = 12699
h px� 
l
%s
*synth2T
R---------------------------------------------------------------------------------
h p
x
� 
l
%s
*synth2T
R---------------------------------------------------------------------------------
h p
x
� 
-
%s
*synth2
Start IO Insertion
h p
x
� 
l
%s
*synth2T
R---------------------------------------------------------------------------------
h p
x
� 
l
%s
*synth2T
R---------------------------------------------------------------------------------
h p
x
� 
?
%s
*synth2'
%Start Flattening Before IO Insertion
h p
x
� 
l
%s
*synth2T
R---------------------------------------------------------------------------------
h p
x
� 
l
%s
*synth2T
R---------------------------------------------------------------------------------
h p
x
� 
B
%s
*synth2*
(Finished Flattening Before IO Insertion
h p
x
� 
l
%s
*synth2T
R---------------------------------------------------------------------------------
h p
x
� 
l
%s
*synth2T
R---------------------------------------------------------------------------------
h p
x
� 
6
%s
*synth2
Start Final Netlist Cleanup
h p
x
� 
l
%s
*synth2T
R---------------------------------------------------------------------------------
h p
x
� 
l
%s
*synth2T
R---------------------------------------------------------------------------------
h p
x
� 
9
%s
*synth2!
Finished Final Netlist Cleanup
h p
x
� 
l
%s
*synth2T
R---------------------------------------------------------------------------------
h p
x
� 
l
%s
*synth2T
R---------------------------------------------------------------------------------
h p
x
� 
�
%s*synth2�
�Finished IO Insertion : Time (s): cpu = 00:00:15 ; elapsed = 00:00:15 . Memory (MB): peak = 2116.773 ; gain = 633.227 ; free physical = 1654 ; free virtual = 12636
h px� 
l
%s
*synth2T
R---------------------------------------------------------------------------------
h p
x
� 
l
%s
*synth2T
R---------------------------------------------------------------------------------
h p
x
� 
=
%s
*synth2%
#Start Renaming Generated Instances
h p
x
� 
l
%s
*synth2T
R---------------------------------------------------------------------------------
h p
x
� 
l
%s
*synth2T
R---------------------------------------------------------------------------------
h p
x
� 
�
%s*synth2�
�Finished Renaming Generated Instances : Time (s): cpu = 00:00:15 ; elapsed = 00:00:15 . Memory (MB): peak = 2116.773 ; gain = 633.227 ; free physical = 1654 ; free virtual = 12636
h px� 
l
%s
*synth2T
R---------------------------------------------------------------------------------
h p
x
� 
l
%s
*synth2T
R---------------------------------------------------------------------------------
h p
x
� 
:
%s
*synth2"
 Start Rebuilding User Hierarchy
h p
x
� 
l
%s
*synth2T
R---------------------------------------------------------------------------------
h p
x
� 
l
%s
*synth2T
R---------------------------------------------------------------------------------
h p
x
� 
�
%s*synth2�
�Finished Rebuilding User Hierarchy : Time (s): cpu = 00:00:15 ; elapsed = 00:00:15 . Memory (MB): peak = 2116.773 ; gain = 633.227 ; free physical = 1654 ; free virtual = 12636
h px� 
l
%s
*synth2T
R---------------------------------------------------------------------------------
h p
x
� 
l
%s
*synth2T
R---------------------------------------------------------------------------------
h p
x
� 
9
%s
*synth2!
Start Renaming Generated Ports
h p
x
� 
l
%s
*synth2T
R---------------------------------------------------------------------------------
h p
x
� 
l
%s
*synth2T
R---------------------------------------------------------------------------------
h p
x
� 
�
%s*synth2�
�Finished Renaming Generated Ports : Time (s): cpu = 00:00:15 ; elapsed = 00:00:15 . Memory (MB): peak = 2116.773 ; gain = 633.227 ; free physical = 1654 ; free virtual = 12636
h px� 
l
%s
*synth2T
R---------------------------------------------------------------------------------
h p
x
� 
l
%s
*synth2T
R---------------------------------------------------------------------------------
h p
x
� 
;
%s
*synth2#
!Start Handling Custom Attributes
h p
x
� 
l
%s
*synth2T
R---------------------------------------------------------------------------------
h p
x
� 
l
%s
*synth2T
R---------------------------------------------------------------------------------
h p
x
� 
�
%s*synth2�
�Finished Handling Custom Attributes : Time (s): cpu = 00:00:15 ; elapsed = 00:00:15 . Memory (MB): peak = 2116.773 ; gain = 633.227 ; free physical = 1654 ; free virtual = 12636
h px� 
l
%s
*synth2T
R---------------------------------------------------------------------------------
h p
x
� 
l
%s
*synth2T
R---------------------------------------------------------------------------------
h p
x
� 
8
%s
*synth2 
Start Renaming Generated Nets
h p
x
� 
l
%s
*synth2T
R---------------------------------------------------------------------------------
h p
x
� 
l
%s
*synth2T
R---------------------------------------------------------------------------------
h p
x
� 
�
%s*synth2�
�Finished Renaming Generated Nets : Time (s): cpu = 00:00:15 ; elapsed = 00:00:15 . Memory (MB): peak = 2116.773 ; gain = 633.227 ; free physical = 1654 ; free virtual = 12636
h px� 
l
%s
*synth2T
R---------------------------------------------------------------------------------
h p
x
� 
l
%s
*synth2T
R---------------------------------------------------------------------------------
h p
x
� 
9
%s
*synth2!
Start Writing Synthesis Report
h p
x
� 
l
%s
*synth2T
R---------------------------------------------------------------------------------
h p
x
� 
W
%s
*synth2?
=
DSP Final Report (the ' indicates corresponding REG is set)
h p
x
� 
�
%s
*synth2~
|+------------+-------------+--------+--------+--------+--------+--------+------+------+------+------+-------+------+------+
h p
x
� 
�
%s
*synth2
}|Module Name | DSP Mapping | A Size | B Size | C Size | D Size | P Size | AREG | BREG | CREG | DREG | ADREG | MREG | PREG | 
h p
x
� 
�
%s
*synth2~
|+------------+-------------+--------+--------+--------+--------+--------+------+------+------+------+-------+------+------+
h p
x
� 
�
%s
*synth2
}|CMX1        | (A*B)'      | 0      | 8      | -      | -      | 25     | 0    | 0    | -    | -    | -     | 0    | 1    | 
h p
x
� 
�
%s
*synth2
}|CMX1        | A*B         | 0      | 8      | -      | -      | 25     | 0    | 0    | -    | -    | -     | 0    | 0    | 
h p
x
� 
�
%s
*synth2
}|CMX2        | A*B         | 0      | 8      | -      | -      | 25     | 0    | 0    | -    | -    | -     | 0    | 0    | 
h p
x
� 
�
%s
*synth2
}|CMX2        | A*B         | 0      | 8      | -      | -      | 25     | 0    | 0    | -    | -    | -     | 0    | 0    | 
h p
x
� 
�
%s
*synth2
}+------------+-------------+--------+--------+--------+--------+--------+------+------+------+------+-------+------+------+

h p
x
� 
/
%s
*synth2

Report BlackBoxes: 
h p
x
� 
8
%s
*synth2 
+-+--------------+----------+
h p
x
� 
8
%s
*synth2 
| |BlackBox name |Instances |
h p
x
� 
8
%s
*synth2 
+-+--------------+----------+
h p
x
� 
8
%s
*synth2 
+-+--------------+----------+
h p
x
� 
/
%s*synth2

Report Cell Usage: 
h px� 
3
%s*synth2
+------+--------+------+
h px� 
3
%s*synth2
|      |Cell    |Count |
h px� 
3
%s*synth2
+------+--------+------+
h px� 
3
%s*synth2
|1     |BUFG    |     1|
h px� 
3
%s*synth2
|2     |CARRY4  |    28|
h px� 
3
%s*synth2
|3     |DSP48E1 |     4|
h px� 
3
%s*synth2
|4     |LUT1    |   109|
h px� 
3
%s*synth2
|5     |LUT2    |     4|
h px� 
3
%s*synth2
|6     |LUT3    |     2|
h px� 
3
%s*synth2
|7     |LUT4    |     2|
h px� 
3
%s*synth2
|8     |LUT5    |    16|
h px� 
3
%s*synth2
|9     |LUT6    |    65|
h px� 
3
%s*synth2
|10    |FDRE    |   354|
h px� 
3
%s*synth2
|11    |IBUF    |     3|
h px� 
3
%s*synth2
|12    |OBUF    |    32|
h px� 
3
%s*synth2
+------+--------+------+
h px� 
3
%s
*synth2

Report Instance Areas: 
h p
x
� 
Q
%s
*synth29
7+------+------------------+-------------------+------+
h p
x
� 
Q
%s
*synth29
7|      |Instance          |Module             |Cells |
h p
x
� 
Q
%s
*synth29
7+------+------------------+-------------------+------+
h p
x
� 
Q
%s
*synth29
7|1     |top               |                   |   620|
h p
x
� 
Q
%s
*synth29
7|2     |  fftp            |fft_processor      |   469|
h p
x
� 
Q
%s
*synth29
7|3     |    cmx1          |CMX1               |    45|
h p
x
� 
Q
%s
*synth29
7|4     |    cmx2          |CMX2               |    80|
h p
x
� 
Q
%s
*synth29
7|5     |    stage4_imag_2 |BF1                |    47|
h p
x
� 
Q
%s
*synth29
7|6     |    stage4_imag_4 |BF1_5              |    47|
h p
x
� 
Q
%s
*synth29
7|7     |    stage4_real_2 |BF1_6              |    47|
h p
x
� 
Q
%s
*synth29
7|8     |    stage4_real_4 |BF1_7              |    47|
h p
x
� 
Q
%s
*synth29
7|9     |  lc1             |Loadable_Counter   |    11|
h p
x
� 
Q
%s
*synth29
7|10    |  lc2             |Loadable_Counter_0 |    11|
h p
x
� 
Q
%s
*synth29
7|11    |  lc3             |Loadable_Counter_1 |    10|
h p
x
� 
Q
%s
*synth29
7|12    |  p2s1            |P2S                |    39|
h p
x
� 
Q
%s
*synth29
7|13    |  p2s2            |P2S_2              |    39|
h p
x
� 
Q
%s
*synth29
7|14    |  pg1             |PG                 |     1|
h p
x
� 
Q
%s
*synth29
7|15    |  pg2             |PG_3               |     1|
h p
x
� 
Q
%s
*synth29
7|16    |  pg3             |PG_4               |     3|
h p
x
� 
Q
%s
*synth29
7+------+------------------+-------------------+------+
h p
x
� 
l
%s
*synth2T
R---------------------------------------------------------------------------------
h p
x
� 
�
%s*synth2�
�Finished Writing Synthesis Report : Time (s): cpu = 00:00:15 ; elapsed = 00:00:15 . Memory (MB): peak = 2116.773 ; gain = 633.227 ; free physical = 1654 ; free virtual = 12636
h px� 
l
%s
*synth2T
R---------------------------------------------------------------------------------
h p
x
� 
`
%s
*synth2H
FSynthesis finished with 0 errors, 1 critical warnings and 8 warnings.
h p
x
� 
�
%s
*synth2�
�Synthesis Optimization Runtime : Time (s): cpu = 00:00:15 ; elapsed = 00:00:15 . Memory (MB): peak = 2116.773 ; gain = 633.227 ; free physical = 1654 ; free virtual = 12636
h p
x
� 
�
%s
*synth2�
�Synthesis Optimization Complete : Time (s): cpu = 00:00:15 ; elapsed = 00:00:15 . Memory (MB): peak = 2116.781 ; gain = 633.227 ; free physical = 1654 ; free virtual = 12636
h p
x
� 
B
 Translating synthesized netlist
350*projectZ1-571h px� 
�
r%sTime (s): cpu = %s ; elapsed = %s . Memory (MB): peak = %s ; gain = %s ; free physical = %s ; free virtual = %s
480*common2
Netlist sorting complete. 2
00:00:00.012
00:00:00.012

2129.6482
0.0002
17682
12749Z17-722h px� 
T
-Analyzing %s Unisim elements for replacement
17*netlist2
32Z29-17h px� 
X
2Unisim Transformation completed in %s CPU seconds
28*netlist2
0Z29-28h px� 
K
)Preparing netlist for logic optimization
349*projectZ1-570h px� 
Q
)Pushed %s inverter(s) to %s load pin(s).
98*opt2
02
0Z31-138h px� 
�
r%sTime (s): cpu = %s ; elapsed = %s . Memory (MB): peak = %s ; gain = %s ; free physical = %s ; free virtual = %s
480*common2
Netlist sorting complete. 2

00:00:002

00:00:002

2277.2382
0.0002
16612
12641Z17-722h px� 
l
!Unisim Transformation Summary:
%s111*project2'
%No Unisim elements were transformed.
Z1-111h px� 
V
%Synth Design complete | Checksum: %s
562*	vivadotcl2

e198d153Z4-1430h px� 
C
Releasing license: %s
83*common2
	SynthesisZ17-83h px� 
~
G%s Infos, %s Warnings, %s Critical Warnings and %s Errors encountered.
28*	vivadotcl2
422
82
12
0Z4-41h px� 
L
%s completed successfully
29*	vivadotcl2
synth_designZ4-42h px� 
�
r%sTime (s): cpu = %s ; elapsed = %s . Memory (MB): peak = %s ; gain = %s ; free physical = %s ; free virtual = %s
480*common2
synth_design: 2

00:00:222

00:00:192

2277.2382	
793.6992
16612
12641Z17-722h px� 
�
%s peak %s Memory [%s] %s12246*common2
synth_design2

Physical2
PSS2=
;(MB): overall = 1472.875; main = 1472.875; forked = 266.562Z17-2834h px� 
�
%s peak %s Memory [%s] %s12246*common2
synth_design2	
Virtual2
VSS2=
;(MB): overall = 2851.328; main = 2277.242; forked = 942.387Z17-2834h px� 
c
%s6*runtcl2G
ESynthesis results are not added to the cache due to CRITICAL_WARNING
h px� 
�
r%sTime (s): cpu = %s ; elapsed = %s . Memory (MB): peak = %s ; gain = %s ; free physical = %s ; free virtual = %s
480*common2
Write ShapeDB Complete: 2

00:00:002

00:00:002

2301.2502
0.0002
16612
12641Z17-722h px� 
�
 The %s '%s' has been generated.
621*common2

checkpoint2H
F/home/sj/Documents/study/asic/project_1/project_1.runs/synth_1/TOP.dcpZ17-1381h px� 
�
Executing command : %s
56330*	planAhead2Q
Oreport_utilization -file TOP_utilization_synth.rpt -pb TOP_utilization_synth.pbZ12-24828h px� 
\
Exiting %s at %s...
206*common2
Vivado2
Thu Mar 13 15:55:11 2025Z17-206h px� 


End Record