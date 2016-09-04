# Edit from simple-wireless.tcl
puts "????"
# Define options
set val(chan)           Channel/WirelessChannel    ;# channel type
set val(prop)           Propagation/TwoRayGround   ;# radio-propagation model
set val(netif)          Phy/WirelessPhy/MIMO       ;# network interface type
set val(mac)            Mac/802_11n                ;# MAC type
set val(ifq)            Queue/Aggr/APriQ           ;# interface queue type
set val(ll)             LL                         ;# link layer type
set val(ant)            Antenna/OmniAntenna        ;# antenna model
set val(ifqlen)         50                         ;# max packet in ifq
set val(nn)             4                          ;# number of mobilenodes
set val(rp)             DSDV                       ;# routing protocol

set time [expr [lindex $argv 0] * 0.00001]
set size [lindex $argv 1]
set RD [lindex $argv 2]
set BA [lindex $argv 3]
set AG [lindex $argv 4]

Queue/Aggr set max_aggr_size_ [expr $AG]         ;# Max Aggregation Size

Mac/802_11n set BlockACKType_ [expr $BA];
Mac/802_11n set RTSThreshold_  500000;
Mac/802_11n set cfb_ 1;

Mac/802_11n set basicRate_ 96e6;
Mac/802_11n set dataRate_ 96e6;

# Initialize Global Variables
set ns_		[new Simulator]
#set tracefd     [open test-[expr $RD]-[expr $BA]-[expr $AG].tr w]
set tracefd [open test.tr w]
set seed [clock clicks -milliseconds]

puts "SEED"
puts $seed

if {$seed < 0} {set seed [expr $seed * -1]}
#ns-random [clock clicks -milliseconds]
ns-random 0

$ns_ use-newtrace
$ns_ trace-all $tracefd
$ns_ eventtrace-all

# set up topography object
set topo       [new Topography]
$topo load_flatgrid 500 500

# Create God
create-god $val(nn)


# configure node

        $ns_ node-config -adhocRouting $val(rp) \
			 -llType $val(ll) \
			 -macType $val(mac) \
			 -ifqType $val(ifq) \
			 -ifqLen $val(ifqlen) \
			 -antType $val(ant) \
			 -propType $val(prop) \
			 -phyType $val(netif) \
			 -channelType $val(chan) \
			 -topoInstance $topo \
			 -agentTrace ON \
			 -routerTrace OFF \
			 -macTrace OFF \
			 -movementTrace OFF			
			 
	for {set i 0} {$i < $val(nn) } {incr i} {
		set node_($i) [$ns_ node]	
		$node_($i) random-motion 0		;# disable random motion
	}


# Provide initial (X,for now Y=Z=0) co-ordinates for mobilenodes

$node_(0) set X_ 5.0
$node_(0) set Y_ 0.0
$node_(0) set Z_ 0.0

$node_(1) set X_ 5.0
$node_(1) set Y_ 300.0
$node_(1) set Z_ 0.0

# 252.0
$node_(2) set X_ 150.0
$node_(2) set Y_ 0.0
$node_(2) set Z_ 0.0

$node_(3) set X_ 150.0
$node_(3) set Y_ 300.0
$node_(3) set Z_ 0.0

$node_(0) NumAntenna 4
$node_(1) NumAntenna 4
$node_(2) NumAntenna 4
$node_(3) NumAntenna 4

$node_(0) MIMOSystem 1
$node_(1) MIMOSystem 1
$node_(2) MIMOSystem 1
$node_(3) MIMOSystem 1

if {$RD > 0} { $node_(0) MacRDRight 2 1; $node_(0) RDRight 1;}
if {$RD > 1} { $node_(1) MacRDRight 3 1; $node_(1) RDRight 1 }
if {$RD > 2} { $node_(2) MacRDRight 0 0.5}
if {$RD > 3} { $node_(3) MacRDRight 1 0.5}

# Setup traffic flow between nodes
# UDP connections between node_(0) and node_(1)

set udp1 [new Agent/UDP]
$udp1 set fid_ 1
set null1 [new Agent/Null]
$ns_ attach-agent $node_(0) $udp1
$ns_ attach-agent $node_(2) $null1
$ns_ connect $udp1 $null1
set cbr1 [new Application/Traffic/CBR]
$cbr1 set packetSize_ $size
$cbr1 set interval_ $time
$cbr1 attach-agent $udp1

set udp2 [new Agent/UDP]
$udp2 set fid_ 2
set null2 [new Agent/Null]
$ns_ attach-agent $node_(2) $udp2
$ns_ attach-agent $node_(0) $null2
$ns_ connect $udp2 $null2
set cbr2 [new Application/Traffic/CBR]
$cbr2 set packetSize_ $size
$cbr2 set interval_ $time
$cbr2 attach-agent $udp2

set udp3 [new Agent/UDP]
$udp3 set fid_ 3
set null3 [new Agent/Null]
$ns_ attach-agent $node_(1) $udp3
$ns_ attach-agent $node_(3) $null3
$ns_ connect $udp3 $null3
set cbr3 [new Application/Traffic/CBR]
$cbr3 set packetSize_ $size
$cbr3 set interval_ $time
$cbr3 attach-agent $udp3

set udp4 [new Agent/UDP]
$udp4 set fid_ 4
set null4 [new Agent/Null]
$ns_ attach-agent $node_(3) $udp4
$ns_ attach-agent $node_(1) $null4
$ns_ connect $udp4 $null4
set cbr4 [new Application/Traffic/CBR]
$cbr4 set packetSize_ $size
$cbr4 set interval_ $time
$cbr4 attach-agent $udp4

puts "??"
# Tell nodes when the simulation ends

for {set i 0} {$i < $val(nn) } {incr i} {
    $ns_ at 55.0 "$node_($i) reset";
}
$ns_ at 5.0 "$cbr1 start"
$ns_ at 30.0 "$cbr1 stop"
$ns_ at 5.1 "$cbr2 start"
$ns_ at 30.0 "$cbr2 stop"
$ns_ at 5.2 "$cbr3 start"
$ns_ at 30.0 "$cbr3 stop"
$ns_ at 5.3 "$cbr4 start"
$ns_ at 30.0 "$cbr4 stop"
$ns_ at 55.1 "stop"
$ns_ at 56.0 "puts \"NS EXITING...\" ; $ns_ halt"
proc stop {} {
    global ns_ tracefd
    $ns_ flush-trace
    close $tracefd
#    exec awk -f hw3.awk hw3.tr
}

puts "Starting Simulation..."
$ns_ run
