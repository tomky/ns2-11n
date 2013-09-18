#include <sys/types.h>
#include <sys/stat.h> 
#include <stdio.h>

#include "config.h"
#ifdef HAVE_NETINET_IN_H
#include <netinet/in.h>
#endif

#include "random.h"
#include "object.h"
#include "trafgen.h"

struct tracerec {
        u_int32_t trec_time; /* inter-packet time (usec) */
	u_int32_t trec_size; /* size of packet (bytes */
	u_int32_t trec_type; /* packet type */ 
	u_int32_t trec_seq;  /* frame sequence number */
};


class myTraceFile : public NsObject {
 public:
	myTraceFile();
	void get_next(int&, struct tracerec&); /* called by TrafficGenerator
						* to get next record in trace.
						*/
	int setup();  /* initialize the trace file */
	int command(int argc, const char*const* argv);
	int get_a();		//added by smallko	
 private:
	void recv(Packet*, Handler*); /* must be defined for NsObject */
        int status_; 
	char *name_;  /* name of the file in which the trace is stored */
	int nrec_;    /* number of records in the trace file */
	struct tracerec *trace_; /* array holding the trace */
	int a_;
};

/* instance of a traffic generator.  has a pointer to the TraceFile
 * object and implements the interval() function.
 */

class myTrafficTrace : public TrafficGenerator {
 public:
	myTrafficTrace();
	int command(int argc, const char*const* argv);
	virtual double next_interval(int &);
 protected:
	void timeout();
	myTraceFile *tfile_;
	struct tracerec trec_;
	int ndx_;
	int f_;		// added by smallko
	int s_;		// added by smallko
	int g_;		// added by smallko
	void init();
};


static class myTraceFileClass : public TclClass {
 public:
	myTraceFileClass() : TclClass("myTracefile") {}
	TclObject* create(int, const char*const*) {
		return (new myTraceFile());
	}
} class_my_tracefile;

myTraceFile::myTraceFile() : status_(0)
{
	a_=0;
}

int myTraceFile::get_a()
{
	return a_;
}

int myTraceFile::command(int argc, const char*const* argv)
{

	if (argc == 3) {
		if (strcmp(argv[1], "filename") == 0) {
			name_ = new char[strlen(argv[2])+1];
			strcpy(name_, argv[2]);
			return(TCL_OK);
		}
	}
	return (NsObject::command(argc, argv));
}

void myTraceFile::get_next(int& ndx, struct tracerec& t)
{
	t.trec_time = trace_[ndx].trec_time;
	t.trec_size = trace_[ndx].trec_size;
	t.trec_type = trace_[ndx].trec_type;
	t.trec_seq  = trace_[ndx].trec_seq;

	if (ndx++ == nrec_){
		ndx = 0;
		a_= 1;
	}
}

int myTraceFile::setup()
{
	tracerec* t;
//	struct stat buf;
	int i;
	unsigned long time, size, type, seq;
	FILE *fp;

	if((fp = fopen(name_, "r")) == NULL) {
		printf("can't open file %s\n", name_);
		return -1;
	}
	
	nrec_ = 0;
	
	while (!feof(fp)){
		fscanf(fp, "%ld%ld%ld%ld", &time, &size, &type, &seq);
		nrec_++;
	}
	
	nrec_=nrec_-2;	
	printf("%d records\n", nrec_);

	rewind(fp);
	trace_ = new struct tracerec[nrec_];

	for (i = 0, t = trace_; i < nrec_; i++, t++){
		fscanf(fp, "%ld%ld%ld%ld", &time, &size, &type, &seq);
		t->trec_time = time;
		t->trec_size = size;
		t->trec_type = type;
		t->trec_seq  =  seq;
	}

	return 0;
}

void myTraceFile::recv(Packet*, Handler*)
{
        /* shouldn't get here */
        abort();
}

/**************************************************************/

static class myTrafficTraceClass : public TclClass {
 public:
	myTrafficTraceClass() : TclClass("Application/Traffic/myTrace") {}
	TclObject* create(int, const char*const*) {
	        return(new myTrafficTrace());
	}
} class_my_traffictrace;

myTrafficTrace::myTrafficTrace()
{
	tfile_ = (myTraceFile *)NULL;
}

void myTrafficTrace::init()
{
	if (tfile_) 
		ndx_ = tfile_->setup();
}

int myTrafficTrace::command(int argc, const char*const* argv)
{
	Tcl& tcl = Tcl::instance();
	
	if (argc == 3) {
		if (strcmp(argv[1], "attach-tracefile") == 0) {
			tfile_ = (myTraceFile *)TclObject::lookup(argv[2]);
			if (tfile_ == 0) {
				tcl.resultf("no such node %s", argv[2]);
				return(TCL_ERROR);
			}
			return(TCL_OK);
		}
	}


	return (TrafficGenerator::command(argc, argv));

}

/* 預定的時間到的時候,就把該Frame往底層傳送 */
void myTrafficTrace::timeout()
{
        if (! running_)
                return;
        
        if (tfile_->get_a()==1){
        	running_=0;
        	return;
	}

        /* send a packet */
	// Note:  May need to set "NEW_BURST" flag in sendmsg() for 
	// signifying a new talkspurt when using vat traces.
	// (see expoo.cc, tcl/ex/test-rcvr.tcl)
	//若是I Frame,則把prio_設為10
	//若是P Frame,則把prio_設為11
	//若是B Frame,則把prio_設為12
	//其它,則根據f_的數字,設定prio_
	if(f_==1){
		agent_->set_prio(10);
	}else if(f_==2){
		agent_->set_prio(11);
	}else if(f_==3){
		agent_->set_prio(12);
	}else {
		agent_->set_prio(f_);
	}
	
	//設定frame type和seq
	agent_->set_frametype(f_);
	agent_->set_frameseq(s_);

	//往底層傳送該frame
	agent_->sendmsg(size_);
	//printf("pktsize:%d pkttype:%d\n",size_, f_);
        /* figure out when to send the next one */
        nextPkttime_ = next_interval(size_);
        /* schedule it */
        timer_.resched(nextPkttime_);
}

double myTrafficTrace::next_interval(int& size)
{
        tfile_->get_next(ndx_, trec_);
	size = trec_.trec_size;
	f_ = trec_.trec_type;
	s_= trec_.trec_seq;
	return(((double)trec_.trec_time)/1000000.0); /* usecs->secs */
}
