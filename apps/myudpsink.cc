#include <stdio.h>
#include <stdlib.h>
#include "myudpsink.h"
#include "ip.h"
#include "udp.h"

static class MyUdpSinkClass : public TclClass {
public:
        MyUdpSinkClass() : TclClass("Agent/MyUdpSink") {}
        TclObject* create(int, const char*const*) {
                return (new MyUdpSink);
        }
} class_myudpsink;

void MyUdpSink::recv(Packet* pkt, Handler*)
{
//        hdr_ip* iph=hdr_ip::access(pkt);
        hdr_cmn* cmh = hdr_cmn::access(pkt);
	
  int numbytes = cmh->size();
	int expect_seq;
//	char buf[100];
	
	//printf("myudpsink(recv):size:%d ip->prio_:%d frametype:%d frameseq:%d\n", numbytes, iph->prio_, cmh->frametype_, cmh->frameseq_);
	if(openfile!=0){
		if(cmh->frameseq_!=cur_seq) {
			expect_seq=cur_seq+1;
			if(expect_seq!=cmh->frameseq_) {
				fprintf(BWFile,"%-16d%-16c%-16d\n", cur_seq, cur_type, size);
				while(expect_seq!=cmh->frameseq_){
					fprintf(BWFile,"%-16d%-16c%-16d\n", expect_seq++, '0', 0);
				}			
			} else {
				if(cur_seq!=0)
					fprintf(BWFile,"%-16d%-16c%-16d\n", cur_seq, cur_type, size);	
			}
					
			size=numbytes;
			cur_seq=cmh->frameseq_;
		
			switch(cmh->frametype_){
			case I: cur_type='I';
				break;
			case P: cur_type='P';
				break;
			case B: cur_type='B';
				break;
			default:
				break;
			}
		} else {
			size+=numbytes;
			cur_seq=cmh->frameseq_;
			switch(cmh->frametype_){
			case I: cur_type='I';
				break;
			case P: cur_type='P';
				break;
			case B: cur_type='B';
				break;
			default:
				break;
			}
		} 	
	}
	
  //fprintf(BWFile,"%-16d%-16d%-16d\n", cmh->frameseq_, cmh->frametype_, cmh->size());
  	if (app_)
               app_->recv(hdr_cmn::access(pkt)->size());

        Packet::free(pkt);
}

int MyUdpSink::command(int argc, const char*const* argv)
{
//	Tcl& tcl = Tcl::instance();
	
	if (strcmp(argv[1], "set_filename") == 0) {
		strcpy(BWfile, argv[2]);
		BWFile = fopen(BWfile, "w");
		openfile=1;
		return (TCL_OK);
	}  
	
	if (strcmp(argv[1], "closefile") == 0) {
		fprintf(BWFile,"%-16d%-16c%-16d\n", cur_seq, cur_type, size);
		return (TCL_OK);
	}
	
	return (Agent::command(argc, argv));
}


MyUdpSink::~MyUdpSink()
{
	fclose(BWFile);
}


