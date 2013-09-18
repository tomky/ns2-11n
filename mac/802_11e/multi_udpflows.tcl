#
# globals and flags
#
set ns [new Simulator]
set AKAROA 0

#
#number of nodes
#
set num_wired_nodes  3
set num_mobile_nodes 4
set num_bs_nodes     1 ;# number of base stations
set num_nodes [expr $num_wired_nodes + $num_mobile_nodes + $num_bs_nodes]
set bs_id $num_wired_nodes
#
# Parameter for wireless nodes
#
set opt(chan)           Channel/WirelessChannel    ;# channel type
set opt(prop)           Propagation/TwoRayGround   ;# radio-propagation model
set opt(netif)          Phy/WirelessPhy            ;# network interface type
set opt(mac)            Mac/802_11e                 ;# MAC type
set opt(ifq)            Queue/DTail/PriQ           ;# interface queue type
set opt(ifqlen)         50
set opt(ll)             LL                         ;# link layer type
set opt(ant)            Antenna/OmniAntenna        ;# antenna model
set opt(ifqlen)         50                        ;# max packet in ifq
set opt(adhocRouting)   DSDV                       ;# routing protocol

set opt(x)		670	;# X dimension of the topography
set opt(y)		670		;# Y dimension of the topography



#
# set up for hierarchical routing
# (needed for routing over a basestation)
#
$ns node-config -addressType hierarchical
AddrParams set domain_num_ 2          ;# domain number
lappend cluster_num 1 1               ;# cluster number for each domain 
AddrParams set cluster_num_ $cluster_num
lappend eilastlevel $num_wired_nodes [expr $num_mobile_nodes + $num_bs_nodes] ;# number of nodes for each cluster             
AddrParams set nodes_num_ $eilastlevel


#
#Open the nam trace file
#
#if {$AKAROA==0 } {
    #set nf [open out.nam w]
    set nf [open /dev/null w]
    $ns namtrace-all-wireless $nf $opt(x) $opt(y)
    set ntr [open out.tr w]
    $ns trace-all $ntr
#}

set chan	[new $opt(chan)]
set topo	[new Topography]
$topo load_flatgrid $opt(x) $opt(y)

# Create God
create-god [expr $num_mobile_nodes + $num_bs_nodes]

#
# creating wired nodes
#
for {set i 0} {$i < $num_wired_nodes} {incr i} {
	set W($i) [$ns node 0.0.$i]
    puts "wired node $i created"
}

#
# creating base station
#

$ns node-config -adhocRouting $opt(adhocRouting) \
                 -llType $opt(ll) \
                 -macType $opt(mac) \
                 -ifqType $opt(ifq) \
                 -ifqLen $opt(ifqlen) \
                 -antType $opt(ant) \
                 -propType $opt(prop)    \
                 -phyType $opt(netif) \
                 -channel $chan      \
                 -topoInstance $topo \
                 -wiredRouting ON \
                 -agentTrace OFF \
                 -routerTrace OFF \
                 -macTrace OFF    \
                 -movementTrace OFF

set BS(0) [$ns node 1.0.0]
$BS(0) random-motion 0
puts "Base-Station node $bs_id created"
#provide some co-ord (fixed) to base station node
$BS(0) set X_ 1.0
$BS(0) set Y_ 2.0
$BS(0) set Z_ 0.0



# 
# creating mobile nodes
#
$ns node-config -wiredRouting OFF
for {set i 0} {$i < $num_mobile_nodes} {incr i} {
                set wl_node_($i) [$ns node 1.0.[expr $i + 1]]	
		$wl_node_($i) random-motion 0		;# disable random motion
    puts "wireless node $i created ..."
    $wl_node_($i) base-station [AddrParams addr2id [$BS(0) node-addr]]
    $wl_node_($i) set X_ [expr $i * 10]
    $wl_node_($i) set Y_ [expr $i * 10]
$wl_node_($i) set Z_ 0.0

}
# linking of root to base-station node
$ns duplex-link $W(0) $BS(0) 10Mb 2ms DropTail

# linking of wired nodes to root node
for {set i 1} {$i < $num_wired_nodes} {incr i} {
    $ns duplex-link $W($i) $W(0) 10Mb 2ms DropTail
}

# telling akaroa which macs to observe
set observed_queues 4
lappend observe_list  2
set numconnections [llength $observe_list]
if {$AKAROA == 1} {
    set ak [new Akaroa]
    $ak AkDeclareParameters [expr $numconnections * $observed_queues]
    puts "numparams= $numconnections"
    #$ak AkObservation 1 $througput
    #$ak AkObservation 2 $througput
    #$ak AkObservation 3 $througput
}
#$ns duplex-link $W([expr $num_wired_nodes - 1]) $BS(0) 10Mb 2ms DropTail

set src_udp0 [new Agent/UDP]
$src_udp0 set class_ 0
$src_udp0 set prio_ 0
set dst_udp0 [new Agent/Null]
$ns attach-agent $wl_node_(1) $src_udp0
$ns attach-agent $W(1) $dst_udp0
set app [new Application/Traffic/CBR]
$app attach-agent $src_udp0
$ns connect $src_udp0 $dst_udp0
$ns at 0.0 "$app start" 

set src_udp1 [new Agent/UDP]
$src_udp1 set class_ 1
$src_udp1 set prio_ 1
set dst_udp1 [new Agent/Null]
$ns attach-agent $wl_node_(1) $src_udp1
$ns attach-agent $W(1) $dst_udp1
set app1 [new Application/Traffic/CBR]
$app1 attach-agent $src_udp1
$ns connect $src_udp1 $dst_udp1
$ns at 0.0 "$app1 start" 

set src_udp2 [new Agent/UDP]
$src_udp2 set class_ 2
$src_udp2 set prio_ 2
set dst_udp2 [new Agent/Null]
$ns attach-agent $wl_node_(1) $src_udp2
$ns attach-agent $W(1) $dst_udp2
set app2 [new Application/Traffic/CBR]
$app2 attach-agent $src_udp2
$ns connect $src_udp2 $dst_udp2
$ns at 0.0 "$app2 start" 

set src_udp3 [new Agent/UDP]
$src_udp3 set class_ 3
$src_udp3 set prio_ 3
set dst_udp3 [new Agent/Null]
$ns attach-agent $wl_node_(1) $src_udp3
$ns attach-agent $W(1) $dst_udp3
set app3 [new Application/Traffic/CBR]
$app3 attach-agent $src_udp3
$ns connect $src_udp3 $dst_udp3
$ns at 0.0 "$app3 start" 

# Define node initial position in nam
for {set i 0} {$i < $num_mobile_nodes} {incr i} {
    $ns initial_node_pos $wl_node_($i) 20
   }

#
# Tell nodes when the simulation ends
#
for {set i 0} {$i < $num_mobile_nodes } {incr i} {
    $ns at 10.0 "$wl_node_($i) reset";
}

if {$AKAROA == 0} {
    $ns at 10000.0 "$BS(0) reset";
    $ns at 10000.0 "$app stop"
    $ns at 10000.0 "$app1 stop"
    $ns at 10000.0 "$app2 stop"
    $ns at 10000.0 "$app3 stop"
    #$ns at 10.0 "$ftp1 stop"
    #$ns at 10.0 "$ftp2 stop"
    $ns at 11000.0 "puts \"NS EXITING...\" ; $ns halt"
}
proc stop {} {
    global ns ntr nf
    close $ntr
    close $nf
}

# run the simulation
$ns run
