/* -*-	Mode:C++; c-basic-offset:8; tab-width:8; indent-tabs-mode:t -*- */
/*
 * Copyright (c) 1997 Regents of the University of California.
 * All rights reserved.
 *
 * Redistribution and use in source and binary forms, with or without
 * modification, are permitted provided that the following conditions
 * are met:
 * 1. Redistributions of source code must retain the above copyright
 *    notice, this list of conditions and the following disclaimer.
 * 2. Redistributions in binary form must reproduce the above copyright
 *    notice, this list of conditions and the following disclaimer in the
 *    documentation and/or other materials provided with the distribution.
 * 3. All advertising materials mentioning features or use of this software
 *    must display the following acknowledgement:
 *	This product includes software developed by the Computer Systems
 *	Engineering Group at Lawrence Berkeley Laboratory.
 * 4. Neither the name of the University nor of the Laboratory may be used
 *    to endorse or promote products derived from this software without
 *    specific prior written permission.
 *
 * THIS SOFTWARE IS PROVIDED BY THE REGENTS AND CONTRIBUTORS ``AS IS'' AND
 * ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT LIMITED TO, THE
 * IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS FOR A PARTICULAR PURPOSE
 * ARE DISCLAIMED.  IN NO EVENT SHALL THE REGENTS OR CONTRIBUTORS BE LIABLE
 * FOR ANY DIRECT, INDIRECT, INCIDENTAL, SPECIAL, EXEMPLARY, OR CONSEQUENTIAL
 * DAMAGES (INCLUDING, BUT NOT LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS
 * OR SERVICES; LOSS OF USE, DATA, OR PROFITS; OR BUSINESS INTERRUPTION)
 * HOWEVER CAUSED AND ON ANY THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT
 * LIABILITY, OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN ANY WAY
 * OUT OF THE USE OF THIS SOFTWARE, EVEN IF ADVISED OF THE POSSIBILITY OF
 * SUCH DAMAGE.
 */

// derived from drop-tail.cc

#include <cmu-trace.h>
#include "aggr.h"

#include <iostream>

static class AggrClass : public TclClass {
 public:
	AggrClass() : TclClass("Queue/Aggr") {}
	TclObject* create(int, const char*const*) {
		return (new Aggr);
	}
} class_aggr;

void Aggr::reset()
{
	Queue::reset();
}

int 
Aggr::command(int argc, const char*const* argv) {

	if (argc==2) {
		if (strcmp(argv[1], "printstats") == 0) {
			print_summarystats();
			return (TCL_OK);
		}
	}
	if (argc == 3) {
		if (!strcmp(argv[1], "packetqueue-attach")) {
			delete q_;
			if (!(q_ = (PacketQueue*) TclObject::lookup(argv[2])))
				return (TCL_ERROR);
			else {
				pq_ = q_;
				return (TCL_OK);
			}
		}
	}
	return Queue::command(argc, argv);
}

/*
 * drop-tail
 */
void Aggr::target_handle(NsObject* targ){
    target_ = targ;
}

void Aggr::recv(Packet* p, Handler *h)
{
    //printf("1:Aggr recv queue length:%d\n",getLen());
    Queue::recv(p,h);
	//printf("2:Aggr recv queue length:%d\n",getLen());
}

void Aggr::enque(Packet* p)
{
    	if (summarystats) {
                Queue::updateStats(qib_?q_->byteLength():q_->length());
	}
	int qlimBytes = qlim_ * mean_pktsize_;
	if ((!qib_ && (q_->length() + 1) >= qlim_) ||
  	(qib_ && (q_->byteLength() + hdr_cmn::access(p)->size()) >= qlimBytes)){
	    
            // if the queue would overflow if we added this packet...
		if (drop_front_) { /* remove from head of queue */
		        q_->enque(p);
			Packet *pp = q_->deque();
			drop(pp);
		} else {
		    drop(p);
		}
	} else {
	    q_->enque(p);
	}
}

// Tomky: for AG
Packet* Aggr::deque()
{
        if (summarystats && &Scheduler::instance() != NULL) {
                Queue::updateStats(qib_?q_->byteLength():q_->length());
        }
		// by Tomky
        Packet *pp = q_->deque();
        
        if (!pp) return pp;

        hdr_cmn* cch = HDR_CMN(pp);

	if (cch->next_hop() <= 0) {
	        pp->aggr_ = 0;
		return pp;
        }

        Packet *p = pp->copy();
        hdr_cmn* ch = HDR_CMN(p);

        //ch->size() = 0;
        p->aggr_ = pp;
	pp->aggr_ = 0;
	int k = 0;

	Packet* tp = pp;
	for(int n = getLen();n>0;n--){
		pp = q_->deque();
		cch = HDR_CMN(pp);
		if (cch->next_hop() == ch->next_hop() && k < 63 && ch->size()+cch->size() < max_aggr_size_){
			tp->aggr_ = pp;
			pp->aggr_ = 0;
			tp = pp;
			ch->size() += cch->size();
//			printf("|");
			k++;
		} else {
			q_->enque(pp);
		}
	}
//	printf(" k=%d ,queue limit:%d, remain:%d\n",k,qlim_,getLen());


        if (p->aggr_->aggr_){
                ch->ptype()=PT_AGGR;
        }
	else{
		Packet::free(p->aggr_);
		p->aggr_=0;
	}
		//printf("queue length: %d pkt_num: %d\n",getLen(),ch->uid());
        return p;
		// by Tomky
	
}
// Tomky: for RD
Packet* Aggr::deque(int dst)
{
//	printf("deque for RD\n");
        if (summarystats && &Scheduler::instance() != NULL) {
                Queue::updateStats(qib_?q_->byteLength():q_->length());
        }
                // by Tomky
        Packet *pp=0,*p=0,*tp;
        hdr_cmn *cch=0,*ch=0;
	int k = 0;
	// Go through all packets to see if there is packets for dst
	for(int n = getLen();n>0;n--){
		pp=q_->deque();
		cch = HDR_CMN(pp);
		if (cch->next_hop() == dst){
			if (p == 0){
				p = pp;
				p->aggr_ = p->copy();
				tp=p->aggr_;
				tp->aggr_ = 0;
				ch = HDR_CMN(p);
			} else if (k < 63 && ch->size()+cch->size() < max_aggr_size_) {
				tp->aggr_ = pp;
				tp = pp;
				tp->aggr_ = 0;
				ch->size() += cch->size();
			} else {
				q_->enque(pp);
				k--;
			}
			k++;		
				
		} else {
			q_->enque(pp);
		}
	}

        if (p)
		if (p->aggr_->aggr_) {
                	ch->ptype()=PT_AGGR;
        	}
	        else{
        	        Packet::free(p->aggr_);
                	p->aggr_=0;
	        }
        return p;
        // by Tomky

}
//Add by hsuan
Packet* Aggr::deque(int dst,int maxSize)
{
//	printf("deque for RD\n");
        if (summarystats && &Scheduler::instance() != NULL) {
                Queue::updateStats(qib_?q_->byteLength():q_->length());
        }
                // by Tomky
        Packet *pp=0,*p=0,*tp;
        hdr_cmn *cch=0,*ch=0;
	int k = 0;
	// Go through all packets to see if there is packets for dst
	for(int n = getLen();n>0;n--){
		pp=q_->deque();
		cch = HDR_CMN(pp);
		if (cch->next_hop() == dst){
			if (p == 0){
				p = pp;
				p->aggr_ = p->copy();
				tp=p->aggr_;
				tp->aggr_ = 0;
				ch = HDR_CMN(p);
			} else if (k < 63 && ch->size()+cch->size() <= maxSize) {
				tp->aggr_ = pp;
				tp = pp;
				tp->aggr_ = 0;
				ch->size() += cch->size();
			} else {
				q_->enque(pp);
				k--;
			}
			k++;		
				
		} else {
			q_->enque(pp);
		}
	}

        if (p)
		if (p->aggr_->aggr_) {
                	ch->ptype()=PT_AGGR;
        	}
	        else{
        	        Packet::free(p->aggr_);
                	p->aggr_=0;
	        }
        return p;
        // by Tomky

}

void Aggr::print_summarystats()
{
	//double now = Scheduler::instance().clock();
        printf("True average queue: %5.3f", true_ave_);
        if (qib_)
                printf(" (in bytes)");
        printf(" time: %5.3f\n", total_time_);
}

void Aggr::setPF(int pf){
    PF = pf;
}
void Aggr::setCW_MIN(int cw_min){
    CW_MIN = cw_min;
}
void Aggr::setCW_MAX(int cw_max){
    CW_MAX = cw_max;
}
void Aggr::setAIFS(int aifs){
    AIFS = aifs;
}

void Aggr::setTXOPLimit(double limit){
    LIMIT = limit;
}

int Aggr::getPF(){
    return PF;
}
int Aggr::getCW_MIN(){
    return CW_MIN;
}

int Aggr::getCW_MAX(){
    return CW_MAX;
}
int Aggr::getAIFS(){
    return AIFS;
}

double Aggr::getTXOPLimit(){
    return LIMIT;
}

int Aggr::getLen(){
    return Queue::length();
}

int Aggr::getLen(int dst){
    int n = 0;
    q_->resetIterator();
    Packet* tmp = q_->getNext();
    hdr_cmn* ch = 0;
    while (tmp){
	ch = HDR_CMN(tmp);
	if (ch->next_hop() == dst){
		n++;
	}
	tmp = q_->getNext();
    }
    return n;
    //return Queue::length();
}

int Aggr::getByteLen(){
    return Queue::byteLength();
}

void Aggr::recvHighPri(Packet *p) {
    target_->recv(p, &qh_);
}

void Aggr::setdrop(NsObject* getdrop_) {
    drop_ = getdrop_;
}
//Add by hsuan
int Aggr::getAggrSize() {
    return max_aggr_size_;
}  
