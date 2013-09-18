#include <stdio.h>
#include <stdlib.h>
#include "myudpsink2.h"
#include "ip.h"
#include "udp.h"
#include "rtp.h"

static class myUdpSink2Class : public TclClass {
public:
        myUdpSink2Class() : TclClass("Agent/myUdpSink2") {}
        TclObject* create(int, const char*const*) {
                return (new myUdpSink2);
        }
} class_myudpsink2;

void myUdpSink2::recv(Packet* pkt, Handler*)
{
        //hdr_ip* iph=hdr_ip::access(pkt);
  	hdr_cmn* hdr=hdr_cmn::access(pkt);
	//hdr_rtp* rtp = hdr_rtp::access(pkt);
	
	pkt_received+=1;
	fprintf(tFile,"%-16f id %-16d udp %-16d\n", Scheduler::instance().clock(), (int)hdr->frame_pkt_id_, hdr->size()-28);
	
  	if (app_)
               app_->recv(hdr_cmn::access(pkt)->size());

        Packet::free(pkt);
}

int myUdpSink2::command(int argc, const char*const* argv)
{
//	Tcl& tcl = Tcl::instance();
	
	if (strcmp(argv[1], "set_trace_filename") == 0) {
		strcpy(tbuf, argv[2]);
		tFile = fopen(tbuf, "w");
		return (TCL_OK);
	}  
	
	if (strcmp(argv[1], "closefile") == 0) {	
		fclose(tFile);
		return (TCL_OK);
	}
	
	if(strcmp(argv[1],"printstatus")==0) {
		print_status();
		return (TCL_OK);
	}
	
	return (Agent::command(argc, argv));
}

void myUdpSink2::print_status()
{
	printf("MyUdpSink2)Total packets received:%ld\n", pkt_received);
}
