//* -*-	Mode:C++; c-basic-offset:8; tab-width:8; indent-tabs-mode:t -*- */
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
#undef NDEBUG
#include <assert.h>
#include "delay.h"
#include "connector.h"
#include "packet.h"
#include "random.h"
#include "mobilenode.h"

#include "arp.h"
//#include "mac.h"
#include "mac-timers_802_11n.h"
#include "mac-802_11n.h"
#include "cmu-trace.h"
#include "apriq.h"

//Add by hsuan
#include <cstdlib>
#include <ctime>

//include the following line when using Akaroa
//#include "akaroa.H"
#define AKAROA 0 
#define AK_INTERVAL 0.01 

inline void
Mac802_11n::set_rx_state(MacState x)
{
	rx_state_ = x;
	check_backoff_timer();
}

inline void
Mac802_11n::set_tx_state(int pri, MacState x)
{
	tx_state_[pri] = x;
}

inline void
Mac802_11n::transmit(Packet *p, double t)
{                
	/*                                                              
         * If I'm transmitting without doing CS, such as when           
         * sending an ACK, any incoming packet will be "missed"         
         * and hence, must be discarded.                                
         */                          
	/* However, in RD this maybe not ACK! : Tomky */
                                   
         if(rx_state_ != MAC_IDLE) {                                     
                //struct hdr_mac802_11n *dh = HDR_MAC802_11N(p);            
                                                                        
                /*assert(dh->dh_fc.fc_type == MAC_Type_Control);          
                assert(dh->dh_fc.fc_subtype == MAC_Subtype_BACK ||
			 dh->dh_fc.fc_subtype == MAC_Subtype_ACK); // Add by Tomky : BA*/
                                                                        
                assert(pktRx_);                                         
                struct hdr_cmn *ch = HDR_CMN(pktRx_);                   
                                                                        
                ch->error() = 1;        /* force packet discard */      
                if (pktRx_->aggr_){
                  Packet* pp = pktRx_->aggr_;
                  hdr_cmn* cch = HDR_CMN(pp);
                  while(pp){
                    cch = HDR_CMN(pp);
                    cch->error() = 1;
                    pp = pp->aggr_;
                  }
                }
        }                                                               
                 
	
        /*                                                              
         * pass the packet on the "interface" which will in turn        
         * place the packet on the channel.                             
         *                                                              
         * NOTE: a handler is passed along so that the Network          
         *       Interface can distinguish between incoming and         
         *       outgoing packets.                                      
         */
	struct hdr_cmn *sd = HDR_CMN(p);

	int prio = LEVEL(p);
	tx_active_ = 1;
	sending = 1;
	check_backoff_timer();
	Packet* pp = p->copy();
	Packet* ppp = pp;
	while(pp->aggr_){
		pp->aggr_ = pp->aggr_->copy();
		pp = pp->aggr_;
	}
	struct hdr_mac802_11n *dh = HDR_MAC802_11N(ppp);
	
	// Add by Tomky : MIMO
	dh->dh_na = num_antenna;
	dh->dh_mimo = mimo_system;
	dh->dh_cr = code_rate;
        
        // Add by Tomky : RD adv
        // Check if the packet is RD?
/*        if (dh->dh_fc.fc_rdg == 1 && sd->uid() != 0){
          if (cfb_dur) {
            rd_right = 1;
            //rdg_r = prio + 1;
          } else { // Or cancel RD because not in cfb
            dh->dh_fc.fc_rdg = 0;
            //rdg_r = 0;
          }
        }*/
        // Add by Tomky : RD
        if (rd_right && !cfb_dur && !rdg_s && sd->uid()) 
          dh->dh_fc.fc_rdg = 1;
          
        printf("node %d: transmit packet %d, rdg=%d, rdg_r=%d\n",index_,sd->uid(),dh->dh_fc.fc_rdg,rdg_r);
	// End of Tomky
	if (sd->uid() != 0){
	  // Add by Tomky : jelly
	  printf("s %d %d %f %d (transmit)\n",index_,sd->uid(),Scheduler::instance().clock(),sd->size());
	}

	downtarget_->recv(ppp, this);

	if(sd->ptype() == PT_CBR || sd->ptype() == PT_EXP) {
	  if(!rtx_[prio]){
	    numbytes_[prio] += sd->size() - phymib_.getHdrLen11();
	  }
	} 
	mhIF_.start(txtime(p));  
	mhSend_.start(t);                      

	                                       
} 

void 
Mac802_11n::check_backoff_timer()
{
	if(is_idle() && mhBackoff_.paused()) {
            mhBackoff_.resume();
	}                                   
	if(! is_idle() && mhBackoff_.busy() && ! mhBackoff_.paused()){  
		mhBackoff_.pause();				       
        }                            
 if(!is_idle() && mhDefer_.busy()) mhDefer_.stop();
}

/* ======================================================================
   Global Variables
   ====================================================================== */

EDCA_PHY_MIB::EDCA_PHY_MIB(Mac802_11n *parent)
{
  /*
   * Bind the phy mib objects.  Note that these will be bound
   * to Mac/802_11n variables
   */
  parent->bind("SlotTime_", &SlotTime);
  parent->bind("SIFS_", &SIFSTime);
  parent->bind("PreambleLength_", &PreambleLength);
  parent->bind("PLCPHeaderLength_", &PLCPHeaderLength);
  parent->bind_bw("PLCPDataRate_", &PLCPDataRate);
}

EDCA_MAC_MIB::EDCA_MAC_MIB(Mac802_11n *parent)
{
  /*
   * Bind the phy mib objects.  Note that these will be bound
   * to Mac/802_11 variables
   */
  
  parent->bind("RTSThreshold_", &RTSThreshold);
  parent->bind("ShortRetryLimit_", &ShortRetryLimit);
  parent->bind("LongRetryLimit_", &LongRetryLimit);
  parent->bind("BlockACKType_",&BACKType);
}

/* ======================================================================
   TCL Hooks for the simulator
   ====================================================================== */
static class Mac802_11nClass : public TclClass {
public:
	Mac802_11nClass() : TclClass("Mac/802_11n") {}
	TclObject* create(int, const char*const*) {
		return (new Mac802_11n());
	}
} class_mac802_11n;


/* ======================================================================
   Mac Class Functions
   ====================================================================== */
Mac802_11n::Mac802_11n() : 
  Mac(), phymib_(this), macmib_(this), mhIF_(this), mhNav_(this), mhRecv_(this), mhSend_(this), mhDefer_(this, phymib_.getSlotTime()), mhSifs_(this, phymib_.getSlotTime()), mhBackoff_(this, phymib_.getSlotTime()), AK(this)
{

	rdg_s = 0;      // Add by Tomky : RD
	rdg_r = 0;      // Add by Tomky : RD
	rd_right = 0;   // Add by Tomky : RD
	
	cfb_times = 0;	// Add by Tomky : RD jelly
	rd_times = 1000;	// Add by Tomky : RD jelly
	rd_dur = 0; 
	
	code_rate = 0; // Add by Tomky : MIMO
	num_antenna = 4; // Add by Tomky : MIMO
	mimo_system = 1;
    for (int i=0;i<1000;i++)//Add by hsuan,initialize rd_p
      rd_p[i] = 0;  
	  
	srand(time(0)); //Add by hsuan,srand should be added here! 
	
        // Pointer to PriQ, Cast in priq.cc
	queue_  = 0;
	
	//flags to control if PriQ Parameters have already been adopted
	AIFSset = 0;
	CWset   = 0;
	for(int i=0; i < MAX_PRI; i++){
	    rdhold[i] = -1; // Add by Tomky : RD
	    pktHold_[i] = 0; // Add by Tomky : BA adv
	    pktTxHold_[i] = 0; // Add by Tomky : RD
	    packets_[i] = 0;
	    pktRTS_[i] = 0;
	    pktCTRL_[i] = 0;
	    pktTx_[i] = 0;
	    tx_state_[i] = MAC_IDLE;
	    ssrc_[i] = slrc_[i] = 0;
	    callback_[i] = 0;
	    numbytes_[i] = 0;
	    rtx_[i] = 0;
	    cw_[i] = 0;
	    cwmin_[i] = 0;
	    cwmax_[i] = 0;
	    aifs_[i] = 0;
	    txop_limit_[i] = 0;
	    start_handle_[i] = 0;
	} 
        jitter  =    1000;
	//jitter =   0;
	throughput = 0;
	interval_ =  AK_INTERVAL;
        if(AKAROA) AK.start();
	
	nav_ =       0.0;
	
	rx_state_ =  MAC_IDLE;
	tx_active_ = 0;    
      	
	idle_time =     0;
	sending =       0;
	cfb_dur =       0;
	cfb_active =    0;
        cfb_broadcast = 0;
        
	levels =     0;
	slotnum =    0;
	pf =         0;
	cw_old =     0;

	sifs_ = phymib_.getSIFS();
	pifs_ = phymib_.getPIFS();
	difs_ = phymib_.getDIFS();
		
        // see (802.11-1999, 9.2.10) 
	eifs_ = phymib_.getDIFS();
	
	eifs_nav_ =  0;
	
	sta_seqno_ =        1;
	cache_ =            0;
	cache_node_count_ = 0;
	
        // chk if basic/data rates are set
	// otherwise use bandwidth_ as default;
	
	Tcl& tcl = Tcl::instance();
	tcl.evalf("Mac/802_11n set basicRate_");
	if (strcmp(tcl.result(), "0") != 0) 
		bind_bw("basicRate_", &basicRate_);
	else
	    basicRate_ = bandwidth_;

	tcl.evalf("Mac/802_11n set dataRate_");
	if (strcmp(tcl.result(), "0") != 0) 
	  bind_bw("dataRate_", &dataRate_);
	else
	  dataRate_ = bandwidth_;
	
	bind("cfb_", &cfb_);
}


int
Mac802_11n::command(int argc, const char*const* argv)
{
	if (argc == 3) {
	    if (strcmp(argv[1], "log-target") == 0) {
		logtarget_ = (NsObject*) TclObject::lookup(argv[2]);
		if(logtarget_ == 0)
		    return TCL_ERROR;
		return TCL_OK;
	    } else if(strcmp(argv[1], "nodes") == 0) {
		if(cache_) return TCL_ERROR;
		cache_node_count_ = atoi(argv[2]);
		cache_ = new Host[cache_node_count_ + 1];
		assert(cache_);
		bzero(cache_, sizeof(Host) * (cache_node_count_+1 ));
		return TCL_OK;
	    } else if(strcmp(argv[1], "MacNumAntenna") == 0) { // Add by Tomky : MIMO
                        num_antenna = atoi(argv[2]);
                        return TCL_OK;
            } else if(strcmp(argv[1], "MacMIMOSystem") == 0) {
			mimo_system = atoi(argv[2]);
			return TCL_OK;
	    } else if(strcmp(argv[1], "RDRight") == 0 ) { // Add by Tomky : RD
			rd_right = atoi(argv[2]);
			return TCL_OK;
            } else if(strcmp(argv[1], "RDTimes") == 0 ) { // Add by Tomky : RD jelly
                        rd_times = atoi(argv[2]);
                        return TCL_OK;
              
	    }
		
        //opower+
        } else if(argc == 4) {
            if (strcasecmp(argv[1], "setMacCodeRate") == 0) {
                        code_rate = atof(argv[2])/atof(argv[3]);
                        printf("\nmac layer:%f\n",code_rate);
                        return TCL_OK;
            }else if (strcmp(argv[1], "MacRDRight") == 0) {//Add by hsuan : RD for different MS
                rd_p[atoi(argv[2])] = atof(argv[3]); 
				printf("MS:%drd_p:%f\n",atoi(argv[2]),rd_p[atoi(argv[2])]);
                return TCL_OK;
            } 
	}
        //+opower
	return Mac::command(argc, argv);
}

/* ======================================================================
   Debugging Routines
   ====================================================================== */

/*
 * dump and packet trace are not adopted to 802.11e yet!
 */

void
Mac802_11n::trace_pkt(Packet *p) {
	struct hdr_cmn *ch = HDR_CMN(p);
	struct hdr_mac802_11n* dh = HDR_MAC802_11N(p);
	u_int16_t *t = (u_int16_t*) &dh->dh_fc;

	fprintf(stderr, "\t[ %2x %2x %2x %2x ] %x %s %d\n",
		*t, dh->dh_duration,
		ETHER_ADDR(dh->dh_da), ETHER_ADDR(dh->dh_sa),
		index_, packet_info.name(ch->ptype()), ch->size());
}

void
Mac802_11n::dump(char *fname)
{
	fprintf(stderr,
		"\n%s --- (INDEX: %d, time: %2.9f)\n",
		fname, index_, Scheduler::instance().clock());

	fprintf(stderr,
		"\ttx_state_: %x, rx_state_: %x, nav: %2.9f, idle: %d\n",
		tx_state_, rx_state_, nav_, is_idle());

	fprintf(stderr,
		"\tpktTx_: %x, pktRx_: %x, pktRTS_: %x, pktCTRL_: %x, callback: %x\n",
		 pktTx_,  pktRx_,  pktRTS_,
		 pktCTRL_,  callback_);

	fprintf(stderr,
		"\tDefer: %d, Backoff: %d (%d), Recv: %d, Timer: %d Nav: %d\n",
		mhDefer_.busy(), mhBackoff_.busy(), mhBackoff_.paused(),
		mhRecv_.busy(), mhSend_.busy(), mhNav_.busy());
	fprintf(stderr,
		"\tBackoff Expire: %f\n",
		mhBackoff_.expire());
}


/* ======================================================================
   Packet Headers Routines
   ====================================================================== */
inline int
Mac802_11n::hdr_dst(char* hdr, int dst )
{
	struct hdr_mac802_11n *dh = (struct hdr_mac802_11n*) hdr;
	//dst = (u_int32_t)(dst);

	if(dst > -2)
		STORE4BYTE(&dst, (dh->dh_da));

	return ETHER_ADDR(dh->dh_da);
}

inline int 
Mac802_11n::hdr_src(char* hdr, int src )
{
	struct hdr_mac802_11n *dh = (struct hdr_mac802_11n*) hdr;
	if(src > -2)
		STORE4BYTE(&src, (dh->dh_sa));
	return ETHER_ADDR(dh->dh_sa);
}

inline int 
Mac802_11n::hdr_type(char* hdr, u_int16_t type)
{
	struct hdr_mac802_11n *dh = (struct hdr_mac802_11n*) hdr;
	if(type)
		STORE2BYTE(&type,(dh->dh_body));
	return GET2BYTE(dh->dh_body);
}


/* ======================================================================
   Misc Routines
   ====================================================================== */
inline int
Mac802_11n::is_idle()
{
    if(rx_state_ != MAC_IDLE){
      idle_time = 0;
      return 0;
    }
    if(sending) {
      idle_time = 0;
      return 0;
    }
    
    if(nav_ > Scheduler::instance().clock()){
      idle_time = 0;
      return 0;
    }
    idle_time = Scheduler::instance().clock();
    return 1;
}

void
Mac802_11n::discard(Packet *p, const char* why)
{
	hdr_mac802_11n* mh = HDR_MAC802_11N(p);
	hdr_cmn *ch = HDR_CMN(p);
//	printf("node %d: discard packet %d because %s\n",index_,ch->uid(),why);

	hdr_mac802_11n* mhh = 0; // Add by Tomky : AG
	Packet* pp = 0; // Add by Tomky : AG
#if 0
	/* old logic 8/8/98 -dam */
	/*
	 * If received below the RXThreshold, then just free.
	 */
	if(p->txinfo_.Pr < p->txinfo_.ant.RXThresh) {
		Packet::free(p);
		return;
	}
#endif // 0

	/* if the rcvd pkt contains errors, a real MAC layer couldn't
	   necessarily read any data from it, so we just toss it now */
	if(ch->error() != 0) {
		Packet::free(p);
		return;
	}

	switch(mh->dh_fc.fc_type) {
	case MAC_Type_Management:
		drop(p, why);
		return;
	case MAC_Type_Control:
		switch(mh->dh_fc.fc_subtype) {
		case MAC_Subtype_RTS:
			if((u_int32_t)ETHER_ADDR(mh->dh_sa) == \
			   (u_int32_t)index_) {
				drop(p, why);
				return;
			}
			/* fall through - if necessary */
		case MAC_Subtype_CTS:
		case MAC_Subtype_ACK:
		case MAC_Subtype_BACK: // Add by Tomky : BA
			if((u_int32_t)ETHER_ADDR(mh->dh_da) == \
			   (u_int32_t)index_) {
				drop(p, why);
				return;
			}
			break;
		default:
			fprintf(stderr, "invalid MAC Control subtype\n");
			exit(1);
		}
		break;
	case MAC_Type_Data:
		switch(mh->dh_fc.fc_subtype) {
		case MAC_Subtype_AData: // Add by Tomky : AG
		
		        pp = p->aggr_;
		        while(pp){
		          mh=HDR_MAC802_11N(pp);
		          p->aggr_ = pp->aggr_;
                          if((u_int32_t)ETHER_ADDR(mh->dh_da) == \
                             (u_int32_t)index_ ||
                             (u_int32_t)ETHER_ADDR(mh->dh_sa) == \
                             (u_int32_t)index_ ||
                             (u_int32_t)ETHER_ADDR(mh->dh_da) == MAC_BROADCAST) {
                                 pp->aggr_ = 0;
                                 drop(pp, why);
                          } else {
                            Packet::free(pp);
                          }
                          pp = p->aggr_; 
                        }
                        p->aggr_ = 0;
                        Packet::free(p);
                        return;
                        
		        break;
		case MAC_Subtype_Data:
			if((u_int32_t)ETHER_ADDR(mh->dh_da) == \
			   (u_int32_t)index_ ||
			   (u_int32_t)ETHER_ADDR(mh->dh_sa) == \
			   (u_int32_t)index_ ||
			   (u_int32_t)ETHER_ADDR(mh->dh_da) == MAC_BROADCAST) {
				drop(p, why);
				return;
			}
			break;
		default:
			fprintf(stderr, "invalid MAC Data subtype\n");
			exit(1);
		}
		break;
	default:
		fprintf(stderr, "invalid MAC type (%x)\n", mh->dh_fc.fc_type);
		trace_pkt(p);
		exit(1);
	}
//	clean_aggr(p);
	Packet::free(p);
}

void
Mac802_11n::capture(Packet *p)
{
	/*k
	 * Update the NAV so that this does not screw
	 * up carrier sense.
	 */
    	set_nav(usec(eifs_ + txtime(p)));
        Packet::free(p);
}

void
Mac802_11n::collision(Packet *p)
{
        hdr_cmn *ch = HDR_CMN(p);
        printf("node %d: Collision %d !\n",index_,ch->uid());
	switch(rx_state_) {
	case MAC_RECV:
		set_rx_state(MAC_COLL);
		/* fall through */
	case MAC_COLL:
		assert(pktRx_);
		assert(mhRecv_.busy());
		/*
		 *  Since a collision has occurred, figure out
		 *  which packet that caused the collision will
		 *  "last" the longest.  Make this packet,
		 *  pktRx_ and reset the Recv Timer if necessary.
		 */
		if(txtime(p) > mhRecv_.expire()) {
			mhRecv_.stop();
			discard(pktRx_, DROP_MAC_COLLISION);
			// Tomky: Collision?
			/*Packet *pp = p;
			while (pp){
			  ch = HDR_CMN(pp);
			  ch->error() = 1;
			  pp = pp->aggr_;
			}*/
			pktRx_ = p;
			mhRecv_.start(txtime(pktRx_));
		}
		else {
			discard(p, DROP_MAC_COLLISION);
		}
		break;
	default:
		assert(0);
	}
}

void
Mac802_11n::tx_resume()
{
//	printf("node %d:tx_resume\n",index_);
	double rTime;
	assert(mhSend_.busy() == 0);
	
	for(int pri = 0; pri < MAX_PRI; pri++){
	    //assert(mhDefer_.defer(pri) == 0);
	    if(!mhDefer_.defer(pri)) {
		if(pktCTRL_[pri]) {
		/*
		 *  Need to send a CTS or ACK.
		 */
		    mhSifs_.start(pri, sifs_);
		} else if(pktRTS_[pri]) {
		    if(mhBackoff_.backoff(pri) == 0) {
		      rTime = (Random::random() % getCW(LEVEL(pktRTS_[pri]))) * phymib_.getSlotTime();
		      mhDefer_.start(pri, getAIFS(LEVEL(pktRTS_[pri]))); 
		    }

                } else if (rdg_s == pri+1 && rdhold[pri] != -1) { // Add by Tomky : RD
                        RDsend(pri);  

		} else if(pktTx_[pri]) {
		    if(mhBackoff_.backoff(pri) == 0) {
			hdr_cmn *ch = HDR_CMN(pktTx_[pri]);
			struct hdr_mac802_11n *mh = HDR_MAC802_11N(pktTx_[pri]);
			if (!(rdg_s && rdhold[pri] != -1)){
				if ((u_int32_t) ch->size() < macmib_.RTSThreshold ||
				    (u_int32_t) ETHER_ADDR(mh->dh_da) == MAC_BROADCAST) {
			  
				  if((u_int32_t) ETHER_ADDR(mh->dh_da) == MAC_BROADCAST) rTime = (Random::random() % (getCW(pri) + 1)) * phymib_.getSlotTime();
				  else rTime = 0;
				    mhDefer_.start(pri, getAIFS(pri) + rTime);
				} else {
				    mhSifs_.start(pri, sifs_); 
				}
			} 
		    }
		} else if(callback_[pri] != 0) {
		  printf("node %d: nothing to do in MAC..\n");
		  rtx_[pri] = 0;
		  Handler *h = callback_[pri];
		  callback_[pri] = 0;
		  h->handle((Event*) 0);
		}
		set_tx_state(pri, MAC_IDLE);
	    }
	}
}

void
Mac802_11n::rx_resume()
{
  assert(pktRx_ == 0);
    assert(mhRecv_.busy() == 0);
    set_rx_state(MAC_IDLE);
}


/* ======================================================================
   Timer Handler Routines
   ====================================================================== */
void
Mac802_11n::backoffHandler(int pri)
{
   if(pktCTRL_[pri]) {
    assert(mhSend_.busy() || mhDefer_.defer(pri));
    return;
  }
  
  if(check_pktRTS(pri) == 0)
    return;
  
  if(check_pktTx(pri) == 0)
    return;
}

void
Mac802_11n::deferHandler(int pri)
{
//	printf("node %d: deferHandler\n",index_);
	assert(pktCTRL_[pri] || pktRTS_[pri] || pktTx_[pri]);

	if(check_pktCTRL(pri) == 0)
		return;

	//assert(mhBackoff_.backoff(pri) == 0); // Add by Tomky : RD // debuging...
	if (mhBackoff_.busy() != 0)
	{
		printf("deferHandler:mhBackoff_ busy!\n");
		return;
	}
	if(check_pktRTS(pri) == 0)
		return;

	if(check_pktTx(pri) == 0)
		return;
}

void
Mac802_11n::navHandler()
{
    eifs_nav_ = 0.0;
    if(is_idle() && mhBackoff_.paused()) {
		mhBackoff_.resume();
    }
}

void
Mac802_11n::recvHandler()
{
    
	recv_timer();
}

void
Mac802_11n::sendHandler()
{
//    Scheduler &s = Scheduler::instance();
    sending = 0;
    check_backoff_timer();
    send_timer();
}


void
Mac802_11n::txHandler()
{
    tx_active_ = 0;
    if(cfb_ && !cfb_broadcast) cfb_active = 0;
    if(cfb_broadcast) cfb_broadcast = 0;
}

void
Mac802_11n::defer_stop(int pri)
{
   mhBackoff_.start(pri, getCW(pri), is_idle());
}

/* sends the throughput every AK_INTERVAL to Akaroa 
 * via AkObservation. This works only if you have 
 * installed Akaroa and the ns-2/Akaroa interface.
 */
void
Mac802_11n::calc_throughput()
{
#if AKAROA > 0
  if(AKAROA){	
    if(index_ > 0){
      //int pri = 2; 
      // jitter is for cases in which the simulation is
      // already in a steady state at the very beginning
      if (jitter >0) {if(jitter >= 10 ) jitter -= 10; }
      for(int pri = 0; pri < 3; pri++){
	if(jitter > 0){
	  throughput = ((numbytes_[pri] * 8.) / interval_) + Random::uniform(0,jitter);
	}else {
	  throughput = (8. * numbytes_[pri]) / interval_;
	}
	if(throughput <= 0) throughput = 0.0001;
	AkObservation((3 * (index_ - 1)) + (pri + 1), throughput);
	//AkObservation((6 * (index_ - 1)) + (pri + 1), throughput);
	//AkObservation((pri + 1), throughput);
	//AkObservation(1, throughput);
	//if(index_ == 1 && pri == 0) {
	cout.precision(16);
	//cout<<"Mac "<<index_<<", now: "<<Scheduler::instance().clock()<<", priority "<<pri<<", throughput: "<<throughput<<", interval_: "<<interval_<<" numbytes " << numbytes_[pri] <<"  jitter "<<jitter<<"\n";
	//}
	numbytes_[pri] = 0; throughput = 0;
      }
      AK.start();
    } 
  }
#endif
}

/* ======================================================================
   The "real" Timer Handler Routines
   ====================================================================== */
void
Mac802_11n::send_timer()
{   
// Scheduler &s = Scheduler::instance();
    for(int pri = 0; pri < MAX_PRI; pri ++){
	switch(tx_state_[pri]) {
	/*
	 * Sent a RTS, but did not receive a CTS.
	 */
	case MAC_RTS:
		RetransmitRTS(pri);
		break;
	/*
	 * Sent a CTS, but did not receive a DATA packet.
	 */
	case MAC_CTS:
		assert(pktCTRL_[pri]);
		Packet::free(pktCTRL_[pri]); pktCTRL_[pri] = 0;
		break;
	/*
	 * Sent DATA, but did not receive an ACK packet.
	 */
	case MAC_SEND:
		RetransmitDATA(pri);
		break;
	/*
	 * Sent an ACK, and now ready to resume transmission.
	 */
	case MAC_ACK:
		assert(pktCTRL_[pri]);
		Packet::free(pktCTRL_[pri]); pktCTRL_[pri] = 0;
		set_tx_state(pri, MAC_IDLE);
		if (rdg_r == pri+1){
			rdg_r = 0;
			if (cfb_dur && cfb_dur <= txop_limit_[pri]) {
				mhSifs_.start(pri, sifs_);
				printf("node %d: end RD, reset CFB timer\n",index_);
				return;
			} else if (cfb_dur > txop_limit_[pri]) {
                                cfb_dur = 0;
                                cfb_broadcast = 0;
                                //assert(mhBackoff_.backoff(pri) == 0); //Tomky : RD
                                rst_cw(pri);
                                mhBackoff_.start(pri, getCW(pri), is_idle());
                                //assert(pktTx_[pri]);
                                //Packet::free(pktTx_[pri]); pktTx_[pri] = 0;
                                printf("node %d: exit cfb, rd_right = %d, rdg_r=%d, cfb_times=%d\n",index_,rd_right,rdg_r,cfb_times-1);
                                  
                                // Add by Tomky : RD jelly
                                printf("e %d %d %f (exit cfb)\n",index_,0,Scheduler::instance().clock());
                                
                                cfb_times = 0;
                                double randtemp=rand()%100;
                                if (pktTx_[pri]){
                                    struct hdr_mac802_11n *mh = HDR_MAC802_11N(pktTx_[pri]);
                                    struct hdr_cmn *ch = HDR_CMN(pktTx_[pri]);
                                    printf("node %d: packet:%d dest:%d rd_p:%f time:%f ",index_,ch->uid(),ETHER_ADDR(mh->dh_da),rd_p[ETHER_ADDR(mh->dh_da)],Scheduler::instance().clock());
                                    if(randtemp<(100*rd_p[ETHER_ADDR(mh->dh_da)])){
                                        printf("RD=1\n");
                                        rd_right = 1 ;
                                        rd_target = ETHER_ADDR(mh->dh_da);
                                     }  else{
                                        rd_right = 0;
                                        rd_target = -1;
                                        printf("RD=0\n");
                                     }
                                }
//                                if (mhDefer_.busy()) mhDefer_.stop(); // Event UID fix?
			        tx_resume();
			        return;
			} else {
			  // Add by Tomky : jelly
			  printf("j %d %f %f (RD done)\n",index_,rd_dur/cfb_dur_temp,Scheduler::instance().clock());
			  rd_dur = 0;
			  cfb_dur_temp = 0;
			}
		}
		break;
	case MAC_IDLE:
		break;
	default:
		assert(0);
	}
    }
//    if (mhDefer_.busy()) mhDefer_.stop(); // Event UID fix?
    if (!cfb_active) tx_resume();
    
}

/* ======================================================================
   Outgoing Packet Routines
   ====================================================================== */
int
Mac802_11n::check_pktCTRL(int pri) 
{

	struct hdr_mac802_11n *mh;
        struct hdr_cmn *ch;
	double timeout;

	if(pktCTRL_[pri] == 0)
		return -1;
	if(tx_state_[pri] == MAC_CTS || tx_state_[pri] == MAC_ACK)
		return -1;

	mh = HDR_MAC802_11N(pktCTRL_[pri]);
        ch = HDR_CMN(pktCTRL_[pri]);
							  
	switch(mh->dh_fc.fc_subtype) {
	/*
	 *  If the medium is not IDLE, don't send the CTS.
	 */
	    case MAC_Subtype_CTS:
	      if(!is_idle()) {
			discard(pktCTRL_[pri], DROP_MAC_BUSY); pktCTRL_[pri] = 0;
			return 0;
	      }
		set_tx_state(pri, MAC_CTS);
		
		/*
		 * timeout:  cts + data tx time calculated by
		 *           adding cts tx time to the cts duration
		 *           minus ack tx time -- this timeout is
		 *           a guess since it is unspecified
		 *           (note: mh->dh_duration == cf->cf_duration)
		 */
		timeout = txtime(phymib_.getCTSlen(), basicRate_)
			+ DSSS_EDCA_MaxPropagationDelay			// XXX
			+ sec(mh->dh_duration)
			+ DSSS_EDCA_MaxPropagationDelay			// XXX
			- sifs_
			- txtime(phymib_.getACKlen(), basicRate_);
		
		break;
		/*
		 * IEEE 802.11 specs, section 9.2.8
		 * Acknowledments are sent after an SIFS, without regard to
		 * the busy/idle state of the medium.
		 */
	    case MAC_Subtype_BACK:
	         set_tx_state(pri, MAC_ACK);
               timeout = txtime(ch->size(), basicRate_);
		break;
	    case MAC_Subtype_ACK:
		set_tx_state(pri, MAC_ACK);
               timeout = txtime(phymib_.getACKlen(), basicRate_);
		break;
	default:
		fprintf(stderr, "check_pktCTRL:Invalid MAC Control subtype\n");
		exit(1);
	}

	transmit(pktCTRL_[pri], timeout);
	return 0;
}

int
Mac802_11n::check_pktRTS(int pri) 
{

	struct hdr_mac802_11n *mh;
	double timeout;

	//assert(mhBackoff_.backoff(pri) == 0); //Tomky : RD
	if(pktRTS_[pri] == 0)
 		return -1;
	//struct hdr_cmn *ch = HDR_CMN(pktRTS_);
	mh = HDR_MAC802_11N(pktRTS_[pri]);

 	switch(mh->dh_fc.fc_subtype) {
	case MAC_Subtype_RTS:
	  if(! is_idle()) {
		    inc_cw(pri); 
		    mhBackoff_.start(pri, getCW(pri), is_idle());

		    return 0;
	  }
		set_tx_state(pri, MAC_RTS);
		timeout = txtime(phymib_.getRTSlen(), basicRate_)
			+ DSSS_EDCA_MaxPropagationDelay			// XXX
			+ sifs_
			+ txtime(phymib_.getCTSlen(), basicRate_)
			+ DSSS_EDCA_MaxPropagationDelay;			// XXX
		break;
	default:
	    fprintf(stderr, "check_pktRTS:Invalid MAC Control subtype\n");
		exit(1);
	}
	transmit(pktRTS_[pri], timeout);
	return 0;
}

int 
Mac802_11n::check_pktTx(int pri)
{

        struct hdr_mac802_11n *mh;
	double timeout;
	//assert(mhBackoff_.backoff(pri) == 0); //Tomky : RD
	
	if(pktTx_[pri] == 0) {
	    return -1;
	}

	mh = HDR_MAC802_11N(pktTx_[pri]);
       	//int len = HDR_CMN(pktTx_)->size();
	switch(mh->dh_fc.fc_subtype) {
	// Add by Tomky : AG
	case MAC_Subtype_AData: 
            /*if(!is_idle()){
              sendRTS(pri, ETHER_ADDR(mh->dh_da));
              inc_cw(LEVEL(pktTx_[pri]));
              mhBackoff_.start(LEVEL(pktTx_[pri]), getCW(LEVEL(pktTx_[pri])), is_idle());
              return 0;
            }*/

            set_tx_state(pri, MAC_SEND);
            if((u_int32_t)ETHER_ADDR(mh->dh_da) != MAC_BROADCAST){
                timeout = txtime(pktTx_[pri])
                            + DSSS_EDCA_MaxPropagationDelay             // XXX
                            + sifs_
                            + DSSS_EDCA_MaxPropagationDelay;            // XXX

		timeout += txtime(getBAlen(), basicRate_);
			
             }
             else{
                timeout = txtime(pktTx_[pri]);
                mh->dh_fc.fc_rdg = 0;
             }
                break;
	// End of Tomky
	case MAC_Subtype_Data:
	    /*if(!is_idle()){
	      sendRTS(pri, ETHER_ADDR(mh->dh_da));
	      inc_cw(LEVEL(pktTx_[pri]));
	      mhBackoff_.start(LEVEL(pktTx_[pri]), getCW(LEVEL(pktTx_[pri])), is_idle());
	      return 0;
	    }*/
	    
	    set_tx_state(pri, MAC_SEND);
	    if((u_int32_t)ETHER_ADDR(mh->dh_da) != MAC_BROADCAST)
		timeout = txtime(pktTx_[pri])
		            + DSSS_EDCA_MaxPropagationDelay		// XXX
			    + sifs_
			    + txtime(phymib_.getACKlen(), basicRate_)
			    + DSSS_EDCA_MaxPropagationDelay;		// XXX
		else{
		    timeout = txtime(pktTx_[pri]);
		    mh->dh_fc.fc_rdg = 0;
		}
		break;
	    default:
	    fprintf(stderr, "check_pktTx:Invalid MAC Control subtype\n");
		//printf("pktRTS:%x, pktCTS/ACK:%x, pktTx:%x\n",pktRTS_, pktCTRL_,pktTx_);
		exit(1);
	}
	transmit(pktTx_[pri], timeout);
	return 0;
}
/*
 * Low-level transmit functions that actually place the packet onto
 * the channel.
 */
void
Mac802_11n::sendRTS(int pri, int dst)
{

	Packet *p = Packet::alloc();
	hdr_cmn* ch = HDR_CMN(p);
	struct rts_frame *rf = (struct rts_frame*)p->access(hdr_mac::offset_);
	
	assert(pktTx_[pri]);
	assert(pktRTS_[pri] == 0);

	/*
	 *  If the size of the packet is larger than the
	 *  RTSThreshold, then perform the RTS/CTS exchange.
	 *
	 *  XXX: also skip if destination is a broadcast
	 */
	if( (u_int32_t) HDR_CMN(pktTx_[pri])->size() < macmib_.RTSThreshold ||
	    (u_int32_t) dst == MAC_BROADCAST) {
		Packet::free(p);
		//p = 0;
		return;
	}

	ch->uid() = 0;
	ch->ptype() = PT_MAC;
	ch->size() = phymib_.getRTSlen();
	ch->iface() = -2;
	ch->error() = 0;

	bzero(rf, MAC_HDR_LEN);

	rf->rf_fc.fc_protocol_version = MAC_ProtocolVersion;
 	rf->rf_fc.fc_type	= MAC_Type_Control;
 	rf->rf_fc.fc_subtype	= MAC_Subtype_RTS;
 	rf->rf_fc.fc_to_ds	= 0;
 	rf->rf_fc.fc_from_ds	= 0;
 	rf->rf_fc.fc_more_frag	= 0;
 	rf->rf_fc.fc_retry	= 0;
 	rf->rf_fc.fc_pwr_mgt	= 0;
 	rf->rf_fc.fc_more_data	= 0;
 	rf->rf_fc.fc_wep	= 0;
 	rf->rf_fc.fc_order	= 0;

	//rf->rf_duration = RTS_DURATION(pktTx_);
	STORE4BYTE(&dst, (rf->rf_ra));
	
	/* store rts tx time */
 	ch->txtime() = txtime(ch->size(), basicRate_);
	
	STORE4BYTE(&index_, (rf->rf_ta));
	/* calculate rts duration field */
	rf->rf_duration = usec(sifs_
			       + txtime(phymib_.getCTSlen(), basicRate_)
			       + sifs_
			       + txtime(pktTx_[pri])
			       + sifs_
			       + txtime(getBAlen(), basicRate_));
	
	
	pktRTS_[pri] = p;
}

void
Mac802_11n::sendCTS(int pri, int dst, double rts_duration)
{

	Packet *p = Packet::alloc();
	hdr_cmn* ch = HDR_CMN(p);
	struct cts_frame *cf = (struct cts_frame*)p->access(hdr_mac::offset_);

	assert(pktCTRL_[pri] == 0);

	ch->uid() = 0;
	ch->ptype() = PT_MAC;
	ch->size() = phymib_.getCTSlen();
	ch->iface() = -2;
	ch->error() = 0;
	//ch->direction() = hdr_cmn::DOWN;
	bzero(cf, MAC_HDR_LEN);

	cf->cf_fc.fc_protocol_version = MAC_ProtocolVersion;
	cf->cf_fc.fc_type	= MAC_Type_Control;
	cf->cf_fc.fc_subtype	= MAC_Subtype_CTS;
 	cf->cf_fc.fc_to_ds	= 0;
 	cf->cf_fc.fc_from_ds	= 0;
 	cf->cf_fc.fc_more_frag	= 0;
 	cf->cf_fc.fc_retry	= 0;
 	cf->cf_fc.fc_pwr_mgt	= 0;
 	cf->cf_fc.fc_more_data	= 0;
 	cf->cf_fc.fc_wep	= 0;
 	cf->cf_fc.fc_order	= 0;
	
	//cf->cf_duration = CTS_DURATION(rts_duration);
	STORE4BYTE(&dst, (cf->cf_ra));
	
	/* store cts tx time */
	ch->txtime() = txtime(ch->size(), basicRate_);
	
	/* calculate cts duration */
	cf->cf_duration = usec(sec(rts_duration)
			       - sifs_
			       - txtime(phymib_.getCTSlen(), basicRate_));
	
	pktCTRL_[pri] = p;
	
}

void
Mac802_11n::sendACK(int pri, int dst)
{

	Packet *p = Packet::alloc();
	hdr_cmn* ch = HDR_CMN(p);
	struct ack_frame *af = (struct ack_frame*)p->access(hdr_mac::offset_);
	assert(pktCTRL_[pri] == 0);

	ch->uid() = 0; // ACK-UID
	ch->ptype() = PT_MAC;
	ch->size() = phymib_.getACKlen();
	ch->iface() = -2;
	ch->error() = 0;
	HDR_IP(p)->prio() = pri; //same priority as data packet
	bzero(af, MAC_HDR_LEN);

	af->af_fc.fc_protocol_version = MAC_ProtocolVersion;
 	af->af_fc.fc_type	= MAC_Type_Control;
 	af->af_fc.fc_subtype	= MAC_Subtype_ACK;
 	af->af_fc.fc_to_ds	= 0;
 	af->af_fc.fc_from_ds	= 0;
 	af->af_fc.fc_more_frag	= 0;
 	af->af_fc.fc_retry	= 0;
 	af->af_fc.fc_pwr_mgt	= 0;
 	af->af_fc.fc_more_data	= 0;
 	af->af_fc.fc_wep	= 0;
 	af->af_fc.fc_order	= 0;

	//Add by Tomky : RD
	if(rdg_s == pri+1 && rdhold[pri] != -1){
		af->af_fc.fc_rdg      = 1;
	} else {
		af->af_fc.fc_rdg      = 0;
	}

        //af->af_duration = ACK_DURATION();
        STORE4BYTE(&dst, (af->af_ra));

 	ch->txtime() = txtime(ch->size(), basicRate_);
	
	/* calculate ack duration */
 	af->af_duration = 0;

	pktCTRL_[pri] = p;
}

void
Mac802_11n::sendDATA(int pri, Packet *p)
{
	int n = 0;
	
	hdr_cmn* ch = HDR_CMN(p);
	struct hdr_mac802_11n* dh = HDR_MAC802_11N(p);
	//assert(pktTx_[pri] == 0); //Tomky : RD

//	printf("h %d %d %f\n",index_,ch->uid(), Scheduler::instance().clock()); // Add by Tomky : RD jelly

	/*
	 * Update the MAC header
	 */
	ch->size() += phymib_.getHdrLen11();
	
	dh->dh_fc.fc_protocol_version = MAC_ProtocolVersion;
	dh->dh_fc.fc_type       = MAC_Type_Data;
	// Add by Tomky : AG
	if (ch->ptype() == PT_AGGR){
		dh->dh_fc.fc_subtype    = MAC_Subtype_AData;
	}else
	{
		dh->dh_fc.fc_subtype    = MAC_Subtype_Data;
	}
	// Change by Tomky : RD adv
	// Add by hsuan:RD probability 
	// Add by Tomky : RD jelly
	if (!cfb_times && !rdg_r && !rdg_s){
          randtemp=rand()%100;
          printf("node %d: packet:%d dest:%d rd_p:%f time:%f ",index_,ch->uid(),ETHER_ADDR(dh->dh_da),rd_p[ETHER_ADDR(dh->dh_da)],Scheduler::instance().clock());
          if(randtemp<(100*rd_p[ETHER_ADDR(dh->dh_da)])){
            printf("RD=1 (sendDATA)\n");
            rd_right = 1 ;
            rd_target = ETHER_ADDR(dh->dh_da);
            }
          else{
            rd_right = 0;
            rd_target = -1;
            printf("RD=0 (sendDATA)\n");
          }
        }
	// Add by Tomky : RD & AG
	// Add by Tomky : RD adv
	// Add by Tomky : RD jelly
	if(rd_right && rd_target == ETHER_ADDR(dh->dh_da)  && cfb_times < rd_times){
		dh->dh_fc.fc_rdg      = 1;
	}
	else
		dh->dh_fc.fc_rdg      = 0;
	
	if (ch->ptype() == PT_AGGR){
        Packet* pp = p;
		while (pp){
		n++;
            struct hdr_mac802_11n* dhh = HDR_MAC802_11N(pp);
            dhh->dh_fc.fc_protocol_version = MAC_ProtocolVersion;
            dhh->dh_fc.fc_type       = MAC_Type_Data;
            dhh->dh_fc.fc_subtype    = MAC_Subtype_AData;

            dhh->dh_fc.fc_to_ds      = 0;
            dhh->dh_fc.fc_from_ds    = 0;
            dhh->dh_fc.fc_more_frag  = 0;
            dhh->dh_fc.fc_retry      = 0;
            dhh->dh_fc.fc_pwr_mgt    = 0;
            dhh->dh_fc.fc_more_data  = 0;
            dhh->dh_fc.fc_wep        = 0;
            dhh->dh_fc.fc_order      = 0;
            pp = pp->aggr_;
		}
		n--;
    	}
	else 
	{
		n = 1;
		dh->dh_fc.fc_to_ds      = 0;
		dh->dh_fc.fc_from_ds    = 0;
		dh->dh_fc.fc_more_frag  = 0;
		dh->dh_fc.fc_retry      = 0;
		dh->dh_fc.fc_pwr_mgt    = 0;
		dh->dh_fc.fc_more_data  = 0;
		dh->dh_fc.fc_wep        = 0;
		dh->dh_fc.fc_order      = 0;
	}
	//printf("%d %d\n",ch->uid(),n);
	BAlen = phymib_.getBACKlen(macmib_.BACKType);

	// End of Tomky
	/* store data tx time */

	//opower e
        if((u_int32_t)ETHER_ADDR(dh->dh_da) != MAC_BROADCAST) {
                /* store data tx time for unicast packets */
                if(code_rate){
                  ch->txtime() = txtime(ch->size(), dataRate_)/code_rate;
                  dh->dh_duration = usec(txtime(getBAlen(), basicRate_)/code_rate
                                       + phymib_.getSIFS());
                }
                else{
                  ch->txtime() = txtime(ch->size(), dataRate_);
                  dh->dh_duration = usec(txtime(getBAlen(), basicRate_)
                                       + phymib_.getSIFS());
   		 }

        } else {
                /* store data tx time for broadcast packets (see 9.6) */
		BAlen = 0;
                //opower e
                if(code_rate)
		    ch->txtime() = txtime(ch->size(), basicRate_)/code_rate;
	       else
		    ch->txtime() = txtime(ch->size(), basicRate_);

                dh->dh_duration = 0;

        }
        //e opower

	pktTx_[pri] = p;
}

/* ======================================================================
   Retransmission Routines
   ====================================================================== */
void
Mac802_11n::RetransmitRTS(int pri)
{

	assert(pktTx_[pri]);
	assert(pktRTS_[pri]);
	assert(mhBackoff_.backoff(pri) == 0);

	macmib_.RTSFailureCount++;

	ssrc_[pri] += 1;			// STA Short Retry Count

	if(ssrc_[pri] >= macmib_.ShortRetryLimit) {
		discard(pktRTS_[pri], DROP_MAC_RETRY_COUNT_EXCEEDED); pktRTS_[pri] = 0;
		/* tell the callback the send operation failed 
		   before discarding the packet */
		hdr_cmn *ch = HDR_CMN(pktTx_[pri]);
		if (ch->xmit_failure_) {
                        /*
                         *  Need to remove the MAC header so that 
                         *  re-cycled packets don't keep getting
                         *  bigger.
                         */
                        ch->size() -= phymib_.getHdrLen11();
                        ch->xmit_reason_ = XMIT_REASON_RTS;
                        ch->xmit_failure_(pktTx_[pri]->copy(),
                                          ch->xmit_failure_data_);
                }
		//printf("(%d)....discarding RTS:%x\n",index_,pktRTS_);
		rst_cw(pri);		
		discard(pktTx_[pri], DROP_MAC_RETRY_COUNT_EXCEEDED); pktTx_[pri] = 0;
		ssrc_[pri] = 0;
		
	} else {
		//printf("(%d)...retxing RTS:%x\n",index_,pktRTS_);
		struct rts_frame *rf;
		rf = (struct rts_frame*)pktRTS_[pri]->access(hdr_mac::offset_);
		rf->rf_fc.fc_retry = 1;

		inc_cw(LEVEL(pktTx_[pri]));
		mhBackoff_.start(LEVEL(pktTx_[pri]), getCW(pri), is_idle());
	}
}

void
Mac802_11n::RetransmitDATA(int pri)
{
        printf("node %d: retransmit data\n",index_);
        // Add by Tomky : RD
	rdg_s = 0;
	rdhold[pri] = -1;

	struct hdr_cmn *ch;
	struct hdr_mac802_11n *mh;
	u_int32_t *rcount, *thresh;

//	assert(mhBackoff_.backoff(pri) == 0);
	
	assert(pktTx_[pri]);
	assert(pktRTS_[pri] == 0);

	ch = HDR_CMN(pktTx_[pri]);
	mh = HDR_MAC802_11N(pktTx_[pri]);
	
	/*
	 *  Broadcast packets don't get ACKed and therefore
	 *  are never retransmitted.
	 */
	if((u_int32_t)ETHER_ADDR(mh->dh_da) == MAC_BROADCAST) {
	  printf("node %d: broadcast packet\n",index_);
	  /*
	   * Backoff at end of TX.
	   */
	  if(!cfb_ || rx_state_ != MAC_IDLE){
		rst_cw(pri);
		mhBackoff_.start(pri, getCW(pri), is_idle());
//		clean_aggr(pktTx_[pri]);
		Packet::free(pktTx_[pri]); pktTx_[pri] = 0;
		printf("node %d: broadcast done, no cfb\n",index_);
		return;
	  } else{
	    // if this is the first packet in cfb, we must take its
	    // duration into account, too.
	    if(cfb_active == 0) {
	      //cout<<"Mac "<<index<<", setting cfb_dur after Broadcast\n";
	      cfb_dur = txtime(pktTx_[pri])
		            + sifs_;
		            //+ txtime(phymib_.getACKlen(), basicRate_);
              cfb_times ++; // Add by Tomky : RD jelly
	    }
	    assert(pktTx_[pri]);
//	    clean_aggr(pktTx_[pri]);
	    Packet::free(pktTx_[pri]); pktTx_[pri] = 0;
	    printf("node %d: broadcast done, into cfb\n",index_);
	    cfb(pri);
	    return;
	    }
	} else if(cfb_) {
		cfb_active = 0;
		cfb_dur = 0;
		cfb_times = 1;
		// Add by Tomky : RD
	}

	macmib_.ACKFailureCount++;
	rtx_[pri] = 1;
	if((u_int32_t) ch->size() <= macmib_.RTSThreshold) {
		rcount = &ssrc_[pri];
		thresh = &macmib_.ShortRetryLimit;
	}
	else {
		rcount = &slrc_[pri];
		thresh = &macmib_.LongRetryLimit;
	}

	(*rcount)++;

	if(*rcount > *thresh) {
	  numbytes_[pri] -= ch->size() - phymib_.getHdrLen11();
	  rtx_[pri] = 0;
	  macmib_.FailedCount++;
	  /* tell the callback the send operation failed 
	     before discarding the packet */
	  hdr_cmn *ch = HDR_CMN(pktTx_[pri]);
	  if (ch->xmit_failure_) {
	    ch->size() -= phymib_.getHdrLen11();
	    ch->xmit_reason_ = XMIT_REASON_ACK;
	    ch->xmit_failure_(pktTx_[pri]->copy(),
			      ch->xmit_failure_data_);
                }
	  rst_cw(pri);
          // Add by Tomky : AG
	  discard(pktTx_[pri], DROP_MAC_RETRY_COUNT_EXCEEDED); pktTx_[pri] = 0;
	  //printf("(%d)DATA discarded: count exceeded\n",index_);
	  *rcount = 0;
		
		
	}
	else {
		struct hdr_mac802_11n *dh;
		dh = HDR_MAC802_11N(pktTx_[pri]);
		dh->dh_fc.fc_retry = 1;
		//dh->dh_fc.fc_rdg = 0; // Add by Tomky : RD adv
		//rd_right = 0;
		sendRTS(pri, ETHER_ADDR(mh->dh_da));
//		printf("(%d)retxing data:%x..sendRTS..pri:%d\n",index_,pktTx_,pri);
		inc_cw(LEVEL(pktTx_[pri]));
		mhBackoff_.start(pri, getCW(pri), is_idle());
	}
}

/* ======================================================================
   Incoming Packet Routines
   ====================================================================== */
void
Mac802_11n::send(Packet *p, Handler *h)

{

        int pri = LEVEL(p);
	start_handle_[pri]=Scheduler::instance().clock();
	double rTime;
	struct hdr_mac802_11n* dh = HDR_MAC802_11N(p);

	/* 
	 * drop the packet if the node is in sleep mode
	 XXX sleep mode can't stop node from sending packets
	 */
	EnergyModel *em = netif_->node()->energy_model();
	if (em && em->sleep()) {
		em->set_node_sleep(0);
		em->set_node_state(EnergyModel::INROUTE);
	}
	callback_[pri] = h;
	sendDATA(pri, p); // framing and calculation of  tx Duration 
	sendRTS(pri, ETHER_ADDR(dh->dh_da)); //check whether size exceeds RTSthreshold 

	/*
	 * Assign the data packet a sequence number.
	 */
	dh->dh_scontrol = sta_seqno_++;

	/*
	 *  If the medium is IDLE, we must wait for a DIFS
	 *  Space before transmitting.
	 */
//        assert(mhDefer_.defer(pri) == 0); 
	
	if(mhBackoff_.backoff(pri) == 0) { //Mac can be still  in post-backoff
	  if(is_idle()) { 
			/*
			 * If we are already deferring, there is no
			 * need to reset the Defer timer.
			 */
			if(mhDefer_.defer(pri) == 0) {
			  rTime = ((Random::random() % getCW(LEVEL(p))) * phymib_.getSlotTime());
			    mhDefer_.start(LEVEL(p), getAIFS(LEVEL(p))); // + rTime);// - phymib_.getSlotTime());
				

			}			
		}
	/*
	 * If the medium is NOT IDLE, then we start
	 * the backoff timer.
	 */
		else {
		    mhBackoff_.start(LEVEL(p),getCW(LEVEL(p)), is_idle());
		}
	}  
	    
}

void
Mac802_11n::recv(Packet *p, Handler *h)
{
   	struct hdr_cmn *hdr = HDR_CMN(p);
	/*
	 * Sanity Check
	 */
	//assert(initialized());

	/*
	 *  Handle outgoing packets.
	 */
	if(hdr->direction() == hdr_cmn::DOWN) {
//            printf("h %d %d %f\n",index_,hdr->uid(),Scheduler::instance().clock());
	    send(p, h);
	    return;
        }
	/*
	 *  Handle incoming packets.
	 *
	 *  We just received the 1st bit of a packet on the network
	 *  interface.
	 *
	 */
	/*
	 *  If the interface is currently in transmit mode, then
	 *  it probably won't even see this packet.  However, the
	 *  "air" around me is BUSY so I need to let the packet
	 *  proceed.  Just set the error flag in the common header
	 *  to that the packet gets thrown away.
	 */
//	Scheduler &s = Scheduler::instance();
        if( tx_active_ ){
        		hdr->error() = 1;
        		if (p->aggr_){
        		  Packet* pp = p->aggr_;
        		  hdr_cmn* cch = HDR_CMN(pp);
        		  while(pp){
        		    cch = HDR_CMN(pp);
        		    cch->error() = 1;
        		    pp = pp->aggr_;
        		  }
        		}
	
	}

	if(rx_state_ == MAC_IDLE) {
		set_rx_state(MAC_RECV);
		assert(!pktRx_);
		pktRx_ = p;

		/*
		 * Schedule the reception of this packet, in
		 * txtime seconds.
		 */
		//printf("%d timer start @ %f!\n",index_,txtime(p));
		mhRecv_.start(txtime(p));
	} else {
		/*
		 *  If the power of the incoming packet is smaller than the
		 *  power of the packet currently being received by at least
                 *  the capture threshold, then we ignore the new packet.
		 */
		if(pktRx_->txinfo_.RxPr / p->txinfo_.RxPr >= p->txinfo_.CPThresh) {
				capture(p);
		} else {
			collision(p);
		}
	} 
}

void
Mac802_11n::recv_timer()
{ 
//       printf("node %d: recv timer\n",index_);
//    Scheduler &s = Scheduler::instance();
	u_int32_t src; 
	hdr_cmn *ch = HDR_CMN(pktRx_);
	hdr_mac802_11n *mh = HDR_MAC802_11N(pktRx_);

	u_int32_t dst = ETHER_ADDR(mh->dh_da);
	// XXX debug
	//struct cts_frame *cf = (struct cts_frame*)pktRx_->access(hdr_mac::offset_);
	//u_int32_t src = ETHER_ADDR(mh->dh_sa);
	
	u_int8_t  type = mh->dh_fc.fc_type;
	u_int8_t  subtype = mh->dh_fc.fc_subtype;

	assert(pktRx_);
	assert(rx_state_ == MAC_RECV || rx_state_ == MAC_COLL);
	
        /*
         *  If the interface is in TRANSMIT mode when this packet
         *  "arrives", then I would never have seen it and should
         *  do a silent discard without adjusting the NAV.
         */
        if(tx_active_) {
            // 2009/02/14 Happy Valentines Day...
                printf("node %d: TX ACTIVE, clear %d\n",index_,ch->uid());
                Packet::free(pktRx_);
                
                goto done;
        }

	/*
	 * Handle collisions.
	 */
	if(rx_state_ == MAC_COLL) {
            printf("node %d: COL %d\n",index_,ch->uid());
	    discard(pktRx_, DROP_MAC_COLLISION);
	    set_nav(usec(eifs_));
	    eifs_nav_ = eifs_;
	    
	    goto done;
	}
	
	/*
	 * Check to see if this packet was received with enough
	 * bit errors that the current level of FEC still could not
	 * fix all of the problems - ie; after FEC, the checksum still
	 * failed.
	 */
	if( ch->error() && subtype != MAC_Subtype_AData ) {
	    Packet::free(pktRx_);
	    set_nav(usec(eifs_));
	    eifs_nav_ = eifs_;
	    goto done;
	}
	// Add by Tomky : BA
	if ( ch->error() && subtype == MAC_Subtype_AData ) {
	  printf("node %d: Check %d all error\n",index_,ch->uid());
	  Packet* pp = pktRx_->aggr_;
	  hdr_cmn* cch = HDR_CMN(pp);
	  int allerror = 1;
          if (pktRx_->aggr_->aggr_ == 0) { // Tomky: a collision bug fix
            printf("x (collision)");
            allerror = 1;
          } else {
	                                
            while(pp) {
    	      cch = HDR_CMN(pp);
    	      if (!cch->error()) {
	        allerror = 0;
	        printf("o");
              } else {
	        printf("x");
	      }
	      pp = pp->aggr_;
            }
          }
	  printf("\n");
	  
	  if (allerror){
	    Packet::free(pktRx_);
	    set_nav(usec(eifs_));
	    eifs_nav_ = eifs_;
	    goto done;
	  }
	}
	
	/*
	 * avoid Nav-reset bug:
	 * if received packet has no  errors and had no collision, but nav 
	 * was set due to an earlier collision, nav has to be reset!
	 */
        if(mhNav_.busy()) reset_eifs_nav();
	
	/*
	 * IEEE 802.11 specs, section 9.2.5.6
	 *	- update the NAV (Network Allocation Vector)
	 */
	if(dst != (u_int32_t)index_) {
	    set_nav(mh->dh_duration);
	}

        /* tap out - */
        if (tap_ && type == MAC_Type_Data &&
            (MAC_Subtype_Data == subtype || MAC_Subtype_AData == subtype) ) 
		tap_->tap(pktRx_);
	/*
	 * Adaptive Fidelity Algorithm Support - neighborhood infomation 
	 * collection
	 *
	 * Hacking: Before filter the packet, log the neighbor node
	 * I can hear the packet, the src is my neighbor
	 */
	if (netif_->node()->energy_model() && 
	    netif_->node()->energy_model()->adaptivefidelity()) {
		src = ETHER_ADDR(mh->dh_sa);
		netif_->node()->energy_model()->add_neighbor(src);
	}
	/*
	 * Address Filtering
	 */
	if(dst != (u_int32_t)index_ && dst != MAC_BROADCAST) {
		/*
		 *  We don't want to log this event, so we just free
		 *  the packet instead of calling the drop routine.
		 */
		discard(pktRx_, "---");
		goto done;
	}

	switch(type) {

	case MAC_Type_Management:
		discard(pktRx_, DROP_MAC_PACKET_ERROR);
		goto done;
		break;

	case MAC_Type_Control:
		switch(subtype) {
		case MAC_Subtype_RTS:
			recvRTS(pktRx_);
			break;
		case MAC_Subtype_CTS:
			recvCTS(pktRx_);
			break;
		case MAC_Subtype_BACK:
			recvBACK(pktRx_);
			break;
		case MAC_Subtype_ACK:
			recvACK(pktRx_);
			break;
		default:
			fprintf(stderr,"recvTimer1:Invalid MAC Control Subtype %x\n",
				subtype);
			exit(1);
		}
		break;
	case MAC_Type_Data:
		switch(subtype) {
		case MAC_Subtype_AData:
		case MAC_Subtype_Data: // Add by Tomky : AG
			recvDATA(pktRx_);
			break;
		default:
			fprintf(stderr, "recv_timer2:Invalid MAC Data Subtype %x\n",
				subtype);
			exit(1);
		}
		break;
	default:
		fprintf(stderr, "recv_timer3:Invalid MAC Type %x\n", subtype);
		exit(1);
	}
 done:
	pktRx_ = 0;
	rx_resume();
}


void
Mac802_11n::recvRTS(Packet *p)
{
        int pri = LEVEL(p);
	struct rts_frame *rf = (struct rts_frame*)p->access(hdr_mac::offset_);

	if(tx_state_[pri] != MAC_IDLE) {
		discard(p, DROP_MAC_BUSY);
		return;
	}

	/*
	 *  If I'm responding to someone else, discard this RTS.
	 */
	if(pktCTRL_[pri]) {
		discard(p, DROP_MAC_BUSY);
		return;
	}

	sendCTS(pri, ETHER_ADDR(rf->rf_ta), rf->rf_duration);
//	if (mhDefer_.busy()) mhDefer_.stop(); // Event UID fix?
    	tx_resume();

	mac_log(p);
}

/*
 * txtime()	- pluck the precomputed tx time from the packet header
 */
double
Mac802_11n::txtime(Packet *p)
 {
	 struct hdr_cmn *ch = HDR_CMN(p);
	 double t = ch->txtime();
	 if (t < 0.0) {
		 drop(p, "XXX");
 		exit(1);
	 }
	 return t;
 }

 
/*
 * txtime()	- calculate tx time for packet of size "psz" bytes 
 *		  at rate "drt" bps
 */
double
Mac802_11n::txtime(double psz, double drt)
{
  double dsz = psz - phymib_.getPLCPhdrLen();
  int plcp_hdr = phymib_.getPLCPhdrLen() << 3;
  int datalen = (int)dsz << 3;
  
  double t = (((double)plcp_hdr)/phymib_.getPLCPDataRate()) + (((double)datalen)/drt);
  return(t);
}



void
Mac802_11n::recvCTS(Packet *p)
{
        int pri = LEVEL(p);
	if(tx_state_[pri] != MAC_RTS) {
		discard(p, DROP_MAC_INVALID_STATE);
		return;
	}
	assert(pktRTS_[pri]);
	Packet::free(pktRTS_[pri]); pktRTS_[pri] = 0;

	assert(pktTx_[pri]);
	// debug
	//struct hdr_mac802_11 *mh = HDR_MAC802_11(pktTx_);
	//printf("(%d):recvCTS:pktTx_-%x,mac-subtype-%d & pktCTS_:%x\n",index_,pktTx_,mh->dh_fc.fc_subtype,p);
	
	mhSend_.stop();

	/*
	 * The successful reception of this CTS packet implies
	 * that our RTS was successful.  Hence, we can reset
	 * the Short Retry Count and the CW.
	 */
	//ssrc_ = 0;
	//rst_cw();
	//	if(mhDefer_.busy()) mhDefer_.stop();
//	if (mhDefer_.busy()) mhDefer_.stop(); // Event UID fix?
	tx_resume();

	mac_log(p);
}

void
Mac802_11n::recvDATA(Packet *p)
{
        int pri = LEVEL(p);
	struct hdr_mac802_11n *dh = HDR_MAC802_11N(p);
	struct hdr_cmn *ch = HDR_CMN(p);

	Packet* pp = 0;
/*	if (macmib_.BACKType > 2){
	  if (pktHold_) {
	    pp = pktHold_->aggr_;
          } else {
            pktHold_=p->copy();
          }
	} */
	printf("node %d: packet %d from node %d received!\n",index_,ch->uid(),ETHER_ADDR(dh->dh_sa));
	// Add by Tomky : jelly
	u_int32_t dst, src, size;
	u_int8_t  subtype = dh->dh_fc.fc_subtype;

	{	struct hdr_cmn *ch = HDR_CMN(p);

		dst = ETHER_ADDR(dh->dh_da);
		src = ETHER_ADDR(dh->dh_sa);
		size = ch->size();

		/*
		 * Adjust the MAC packet size - ie; strip
		 * off the mac header
		 */
		ch->size() -= phymib_.getHdrLen11();
		ch->num_forwards() += 1;
	}

        // Add by Tomky : RD
        if (rd_right && rdg_r && cfb_dur){
                cfb_dur +=  txtime(p)
                            + sifs_
                            + txtime(phymib_.getACKlen(), basicRate_);
        }
        if (rdg_r) {
          rd_dur += txtime(p) + sifs_ + txtime(phymib_.getACKlen(), basicRate_);
          if (!cfb_dur){
            cfb_dur_temp += txtime(p) + sifs_ + txtime(phymib_.getACKlen(), basicRate_);
          }
        }
        if (cfb_dur >= txop_limit_[pri]) printf("Error! Reach TXOP limit cfb=%f\n",cfb_dur);

        // Add by Tomky : RD
        if( dh->dh_fc.fc_rdg == 1 ){
                if ((int)src != index_){
                 printf("node %d: got %d's RD right!!\n",index_,src);
                 struct hdr_cmn *cch;
                 if (pktTx_[pri]) cch = HDR_CMN(pktTx_[pri]);
		 if (pktTx_[pri] && cch->next_hop() == (int)(src)){
                        printf("node %d: packet %d already in pktTx, dest:%d\n",index_,cch->uid(),cch->next_hop());
                                struct hdr_mac802_11n* mh;
                                mh = HDR_MAC802_11N(pktTx_[pri]);
                                mh->dh_fc.fc_rdg = 0;
                                rdg_s = pri+1;
                                rdhold[pri] = src;
                        /*else {
				rdg_s = 0;
				rdhold[pri] = -1;
			} */
                 } else if (queue_->pri_[pri].getLen(src) > 0) {
                        printf("node %d: some data in queue is to %d\n",index_,src);
                        rdg_s = pri+1;
                        rdhold[pri] = src;
                 } else {
                        printf("node %d: No data for RD...\n",index_);
                        rdhold[pri] = -1;
                        rdg_s = 0;
                 }
                } else {
                        rdg_s = 0;
			rdhold[pri] = -1; 
                }
        }

	/*
	 *  If we sent a CTS, clean up...
	 */
	if(dst != MAC_BROADCAST) {
		if(size >= macmib_.RTSThreshold) {
			if (tx_state_[pri] == MAC_CTS) {
	                        // Add by Tomky : BA
	                        if (subtype == MAC_Subtype_Data){
	                                if ( ch->error() ){
        	                                discard(p, DROP_MAC_COLLISION);
                        	                return;
                                	} else {
						assert(pktCTRL_[pri]);
        	        	                Packet::free(pktCTRL_[pri]); pktCTRL_[pri] = 0;
	        	                        mhSend_.stop();

						sendACK(pri,src);
//						if (mhDefer_.busy()) mhDefer_.stop(); // Event UID fix?
	                                        tx_resume();
        	                        }
                	        } else {
	                                if (!macmib_.BACKType){
        	                                if ( ch->error() ){
                	                                discard(p, DROP_MAC_COLLISION);
                                	                rdhold[pri] = -1;
                                        	        rdg_s = 0;
                                                	return;
	                                        } else {
                		       	        	assert(pktCTRL_[pri]);
	                        		        Packet::free(pktCTRL_[pri]); pktCTRL_[pri] = 0;
        	                        		mhSend_.stop();

        	                                        sendACK(pri, src);
//        	                                        if (mhDefer_.busy()) mhDefer_.stop(); // Event UID fix?
                	                                tx_resume();
                        	                }
                                	} else {
                                	        if (sendBACK(pri,p )) {
                                	                
//                                	                if (mhDefer_.busy()) mhDefer_.stop(); // Event UID fix?
                                        	        tx_resume();

	                                        } else {
        	                                        rdhold[pri] = -1;
                	                                rdg_s = 0;
                        	                        return;
                                	        }
	                                }
        	                }

			}
			else {
				discard(p, DROP_MAC_BUSY);
				printf("(%d)..discard DATA\n",index_);
				return;
			}
		}
		/*
		 *  We did not send a CTS and there's no
		 *  room to buffer an ACK.
		 */
		else {
			if(pktCTRL_[pri]) {
				discard(p, DROP_MAC_BUSY);
				return;
			}
			
                        if (subtype == MAC_Subtype_Data){
                                if (ch->error()){
                                        discard(p, DROP_MAC_COLLISION);
                                        rdhold[pri] = -1;
                                        rdg_s = 0;
                                        return;
                                } else {
                                        sendACK(pri, src);
                        		if(mhSend_.busy() == 0 && mhSifs_.busy() == 0){
//                        		    if (mhDefer_.busy()) mhDefer_.stop(); // Event UID fix?
		                            tx_resume();
					}

                                }
                        } else {
                                if (!macmib_.BACKType){
                                        if (ch->error()){
                                                rdhold[pri] = -1;
                                                rdg_s = 0;
                                                discard(p, DROP_MAC_COLLISION);
                                                return;
                                        } else {
                                                sendACK(pri, src);
        	                                if(mhSend_.busy() == 0 && mhSifs_.busy() == 0){
//        	                                    if (mhDefer_.busy()) mhDefer_.stop(); // Event UID fix?
                	                            tx_resume();

						}
                                        }
                                } else {
                                        if (sendBACK(pri,p)) {
					  if (mhSifs_.busy() == 0 && mhSend_.busy() == 0) {
//					        if (mhDefer_.busy()) mhDefer_.stop(); // Event UID fix?
                                	        tx_resume();

                                	        }
                                          else{
                                        	rdhold[pri] = -1;
	                                        rdg_s = 0;
	                                        printf("node %d: in fact, sending\n",index_);
					  }
					  
					} else {
					   discard(p,DROP_MAC_COLLISION);
					   rdhold[pri] = -1;
					   rdg_s = 0;
					   return;
					}
					
                                }
                        }

		}
                // End of Tomky
	}

	/* ============================================================
	    Make/update an entry in our sequence number cache.
	   ============================================================ */

	/* Changed by Debojyoti Dutta. This upper loop of if{}else was 
	   suggested by Joerg Diederich <dieder@ibr.cs.tu-bs.de>. 
	   Changed on 19th Oct'2000 */

        if(dst != MAC_BROADCAST) {
                if (src < (u_int32_t) cache_node_count_) {
		            Host *h = &cache_[src];
			
                        if(!p->aggr_ && h->seqno && h->seqno == dh->dh_scontrol) {
			    discard(p, DROP_MAC_DUPLICATE);
			    return;
                        }
			if (p->aggr_) {
			
			        dh=HDR_MAC802_11N(p->aggr_);
                                if ( h->seqno && h->seqno == dh->dh_scontrol) {
                                  discard(p, DROP_MAC_DUPLICATE);
                                  return;
                                }
                                dh=HDR_MAC802_11N(p);
                                
				Packet *pp = p;
				struct hdr_cmn *chh = 0;
				while (pp->aggr_ ){ 
				  chh = HDR_CMN(pp->aggr_);
				  if (chh->error())
				    break;
				  pp=pp->aggr_;
				}
				if (pp != p){
    				  struct hdr_mac802_11n *dhh = HDR_MAC802_11N(pp);
    				  h->seqno = dhh->dh_scontrol;
                                }
			} else {
                        	h->seqno = dh->dh_scontrol;
			}
		    
                } else {
			static int count = 0;
			if (++count <= 10) {
				printf ("MAC_802_11n: accessing MAC cache_ array out of range (src %u, dst %u, size %d)!\n", src, dst, cache_node_count_);
				if (count == 10)
					printf ("[suppressing additional MAC cache_ warnings]\n");
			};
		};
	}
	
        double tx = ch->txtime();

	/*

	 *  Pass the packet up to the link-layer.
	 *  XXX - we could schedule an event to account
	 *  for this processing delay.
	 */
	//p->incoming = 1;
	// XXXXX NOTE: use of incoming flag has been depracated; In order to track direction of pkt flow, direction_ in hdr_cmn is used instead. see packet.h for details. 
	if (subtype != MAC_Subtype_AData){
		uptarget_->recv(p, (Handler*) 0);
		return;
	}
	else {
	// Add by Tomky : AG
	
	    if (!p->aggr_) return;
	
            Packet *ppp = p->aggr_;
            Packet *pp = p->aggr_->aggr_;

            p->aggr_ = 0;
            Packet::free(p);
	
            while(ppp){
		printf("r");
	        struct hdr_cmn *chh = HDR_CMN(ppp);
        	chh->num_forwards() += 1;
	        chh->txtime() = tx;
        	chh->direction() = hdr_cmn::UP;
		ppp->aggr_ = 0;
        	uptarget_->recv(ppp, (Handler*) 0);
	        if (!pp) break;
        	ppp = pp;
	        pp = pp->aggr_;
            }
            printf("\n");
	}

	
}


void
Mac802_11n::recvACK(Packet *p)
{
	printf("node %d: recvACK\n",index_);
       struct hdr_mac802_11n *mh = HDR_MAC802_11N(p);
        u_int32_t src = ETHER_ADDR(mh->dh_sa);
        
        
        int pri = LEVEL(p);
	//u_int32_t src = ETHER_ADDR(mh->dh_sa);
	// Add by Tomky : RD

	if (rdg_s == pri+1){
		printf("node %d: Get ACK (RD), time:%f\n",index_,Scheduler::instance().clock());
		rdg_s = 5;
		rdhold[pri] = -1;
	}

	struct hdr_cmn *ch = HDR_CMN(p);
	if(tx_state_[pri] != MAC_SEND) {
	    discard(p, DROP_MAC_INVALID_STATE);
	return;
	}
	//printf("(%d)...................recving ACK:%x\n",index_,p);
	mhSend_.stop();
	if (mhBackoff_.busy()) mhBackoff_.stop();

	// Add by Tomky : RD
	if(rd_right)
	{
		if(mh->dh_fc.fc_rdg == 1 ){
		  rdg_r = pri+1;
		} else {
		  rdg_r = 0;
		}
	}

	/*
	 * The successful reception of this ACK packet implies
	 * that our DATA transmission was successful.  Hence,
	 * we can reset the Short/Long Retry Count and the CW.
	 */
	if((u_int32_t) ch->size() <= macmib_.RTSThreshold)
		ssrc_[pri] = 0;
	else
		slrc_[pri] = 0;

	/* succesful transmission => give delay
	 * to Akaroa
	 */
	if(rtx_[pri]) rtx_[pri] = 0;
//	double delay=Scheduler::instance().clock() - start_handle_[pri];
	#if AKAROA > 0
	if(AKAROA index_ > 0) {
	  AkObservation((6 * (index_ - 1)) + (pri + 4), delay);
	  start_handle_[pri]=0;
	}
	#endif
	sending = 0;
	check_backoff_timer();
        /*
	 * Backoff before sending again.
	 */

	// Add by Tomky : RD
	/*if(rdg_s == 5){

          if (pktTx_[pri]->aggr_){
		clean_aggr(pktTx_[pri]);
          }
          Packet::free(pktTx_[pri]); pktTx_[pri] = 0;
	  rdg_s = 0;
	  printf("Finish RD, stop!\n");
	  tx_resume();
	*/
	if(!cfb_ || ch->size() > (int)macmib_.RTSThreshold || rdg_s == 5) {
	  //assert(mhBackoff_.backoff(pri) == 0); //Tomky : RD
	  rst_cw(pri);
	  rdg_s = 0;
	  mhBackoff_.start(pri, getCW(pri), is_idle());
	  assert(pktTx_[pri]);
          // Add by Tomky : AG
	  Packet::free(pktTx_[pri]); pktTx_[pri] = 0;
	  // Add by Tomky : RD
	  if (pktTxHold_[pri]) {
	    pktTx_[pri] = pktTxHold_[pri];
	    pktTxHold_[pri] = 0;
	  }
	  printf("node %d: start normal backoff\n",index_);
	  
	  // Add by Tomky : RD jelly
	  
	  printf("r %d %d %f (recvACK, no cfb (RD))\n",index_,ch->uid(),Scheduler::instance().clock());
//	  if (mhDefer_.busy()) mhDefer_.stop(); // Event UID fix?
	  tx_resume();

	}
	else{
	  // if this is the first packet in cfb, we must take its
	  // duration into account, too.
	  if(!cfb_dur) {
	      cfb_times ++; // Add by Tomky : RD jelly
	      cfb_dur = txtime(pktTx_[pri])
		            + sifs_
		            + txtime(phymib_.getACKlen(), basicRate_);
	      
	  }

	  // Add by Tomky : RD
	  if (rd_right){
	  	if(rdg_r){
			//cfb_dur += txtime(pktTx_[pri]) + sifs_ + txtime(phymib_.getACKlen(), basicRate_) + sifs_;
		  }
	  }

	/*  pktRx_=0;
	  rx_resume(); */

	  assert(pktTx_[pri]);
	  // Add by Tomky : AG
//	  if (pktTx_[pri]->aggr_){
//		clean_aggr(pktTx_[pri]);
//	  }
	  Packet::free(pktTx_[pri]); pktTx_[pri] = 0;
	  
	  // Add by Tomky : RD jelly
	  printf("r %d %d %f (recvACK, in cfb)\n",index_,ch->uid(),Scheduler::instance().clock());
	  
          cfb(pri);
	}
	mac_log(p);
}

void Mac802_11n::cfb(int pri)
{
  printf("node %d: in cfb, cfb_dur = %f, cfb_times = %d\n",index_,cfb_dur,cfb_times);
  double timeout;
  struct hdr_mac802_11n *mh;
  struct hdr_cmn *ch;
  // next packet out of queue
  //cout<<"packets in queue:"<<queue_->pri_[pri].getLen()<<"\n";
  // Add by Tomky : RD jelly
//  if((queue_->pri_[pri].getLen() > 0 && rd_target == -1 ) || (rd_target != -1 && queue_->pri_[pri].getLen(rd_target))) {
  if (queue_->pri_[pri].getLen() > 0) {
      Packet* p = 0;
      // Add by Tomky : RD jelly
      /*if (rd_target != -1) {
        p = queue_->pri_[pri].deque(rd_target);
      } else { 
        p = queue_->pri_[pri].deque(); 
      }*/
      // Recover by Tomky : RD
      p = queue_->pri_[pri].deque();
      
      ch = HDR_CMN(p);
      // framing
      sendDATA(pri, p);
      mh = HDR_MAC802_11N(pktTx_[pri]);
      //cout<<"Mac "<<index_<<" in cfb(), pri "<<pri<<", cfb_bytes "<<cfb_bytes<<" + "<<ch->size()<<", cfb_maxbytes "<<cfb_maxbytes_<<"\n";
      printf("node %d: packet %d is sent by cfb, fc_rdg=%d\n",index_,ch->uid(),mh->dh_fc.fc_rdg);
      cfb_times ++; // Add by Tomky : RD jelly
      
      if((u_int32_t)ETHER_ADDR(mh->dh_da) != MAC_BROADCAST) {
	  cfb_dur +=  sifs_ 
                      + txtime(pktTx_[pri])
		      + sifs_;
                      //+ txtime(phymib_.getACKlen(), basicRate_);
	  cfb_broadcast = 0;
	  
          cfb_dur += txtime(getBAlen(), basicRate_);

      } else {
	      cfb_dur += sifs_ 
	              + txtime(pktTx_[pri]);
              cfb_broadcast = 1;
      }
  } else cfb_dur = txop_limit_[pri] + 1; 
 
  if(cfb_dur <= txop_limit_[pri]) {
    // send
    if((u_int32_t)ETHER_ADDR(mh->dh_da) != MAC_BROADCAST)
      timeout = txtime(pktTx_[pri])
	                    + DSSS_EDCA_MaxPropagationDelay	
			    + sifs_
			    + txtime(getBAlen(), basicRate_)
			    + DSSS_EDCA_MaxPropagationDelay;

    else
      timeout = txtime(pktTx_[pri]);    

    cfb_active = 1;

	//Add by Tomky : RD 
	if(rdg_r)
	{
		printf("node %d: have set RD ! we should wait more...\n",index_);
		double rdtime = sifs_;
			//+txtime(pktTx_[pri]) + sifs_ + sifs_;
                rdtime += txtime(getBAlen(), basicRate_);
		//mhSifs_.start(pri, rdtime);
		//mhDefer_.start(pri, rdtime);
		//tx_resume();
	}
	else{
    		mhSifs_.start(pri, sifs_);
	}
	printf("node %d: still in cfb, cfb_dur = %f, limit=%f\n",index_,cfb_dur,txop_limit_[pri]);
  }
  else {
    // Add by Tomky : RD
    if (!rdg_r) {
      printf("j %d %f (exit cfb, output ratio)\n",index_,rd_dur/cfb_dur);
      rd_dur = 0;
      cfb_dur_temp = 0;
    } else {
      cfb_dur_temp = cfb_dur;
    }
    
    cfb_dur = 0;
    cfb_broadcast = 0;
    //assert(mhBackoff_.backoff(pri) == 0); //Tomky : RD
    rst_cw(pri);
    mhBackoff_.start(pri, getCW(pri), is_idle());
//    assert(pktTx_[pri]); // Bug?
//    Packet::free(pktTx_[pri]); pktTx_[pri] = 0;
    printf("node %d: exit cfb, rd_right = %d, rdg_r=%d, cfb_times=%d\n",index_,rd_right,rdg_r,cfb_times-1);

    // Add by Tomky : RD jelly    
    printf("e %d %d %f (exit cfb)\n",index_,ch->uid(),Scheduler::instance().clock());
               
    // Add by Tomky : RD jelly
    // 2008/12/25 Merry Christmas!
    
    cfb_times = 0;
        
    
    double randtemp=rand()%100;
    if (pktTx_[pri]){
      printf("node %d: packet:%d dest:%d rd_p:%f time:%f ",index_,ch->uid(),ETHER_ADDR(mh->dh_da),rd_p[ETHER_ADDR(mh->dh_da)],Scheduler::instance().clock());
      if(randtemp<(100*rd_p[ETHER_ADDR(mh->dh_da)])){
         printf("RD=1\n");
         rd_right = 1 ;
         rd_target = ETHER_ADDR(mh->dh_da);
      }  else{
         rd_right = 0;
         rd_target = -1;
         printf("RD=0\n");
      }                                     
    }
//    if (mhDefer_.busy()) mhDefer_.stop(); // Event UID fix?
    tx_resume();
  }
}

// request parameters for each priority from the corresponding queues
double Mac802_11n::getAIFS(int pri)
{
    if(!AIFSset){
      levels = queue_->getLevels();
      for(int i = 0; i < levels; i++ ){
	slotnum = queue_->pri_[i].getAIFS();
	aifs_[i] = sifs_ + (slotnum * phymib_.getSlotTime());
	txop_limit_[i] = queue_->pri_[i].getTXOPLimit();
	//	    cout<<"Mac "<<index_<<", pri: "<<i<<", txop_limit:"<<txop_limit_[i]<<"\n";
      }
      AIFSset = 1;
    }
    return aifs_[pri];
}


int Mac802_11n::getCW(int level)
{
    if(!CWset){
	levels = queue_->getLevels();
	for(int i = 0; i < levels; i++ ){
	    cw_[i] = queue_->pri_[i].getCW_MIN();
	    cwmin_[i] = queue_->pri_[i].getCW_MIN(); 
	    cwmax_[i] = queue_->pri_[i].getCW_MAX(); 
	}
	CWset = 1;
    }
    return cw_[level];
}

void
Mac802_11n::setQ(APriQ* priqueue){
    queue_ = priqueue;
}

inline void 
Mac802_11n::reset_eifs_nav() {
  if (eifs_nav_ > 0) {
    double now = Scheduler::instance().clock();
    
    assert(nav_ > now);
    assert(mhNav_.busy());
    
    mhNav_.stop();
    nav_ -= eifs_nav_;
    eifs_nav_ = 0.0;
    if (nav_ > now) {
      mhNav_.start(nav_ - now);
    } else {
      nav_ = now;
      check_backoff_timer();
    }
  }
}


bool Mac802_11n::inc_retryCounter(int pri) {
  u_int32_t *rcount, *thresh;
  struct hdr_cmn *ch = HDR_CMN(pktTx_[pri]);
  if((u_int32_t) ch->size() <= macmib_.RTSThreshold) {
    ssrc_[pri]++;
    rcount = &ssrc_[pri];
    thresh = &macmib_.ShortRetryLimit;
  }
  else {
    slrc_[pri]++;
    rcount = &slrc_[pri];
    thresh = &macmib_.LongRetryLimit;
  }
  if(*rcount > *thresh) {
    rtx_[pri] = 0;
    macmib_.FailedCount++;
    rst_cw(pri);
    discard(pktTx_[pri], DROP_MAC_RETRY_COUNT_EXCEEDED); pktTx_[pri] = 0;
    *rcount = 0;
    return 1;
  } else
     return 0;
}


// Add by Tomky : RD

void Mac802_11n::RDsend(int pri) {
        Scheduler &s = Scheduler::instance();

	printf("node %d: RDsend to %d, time:%f:",index_,rdhold[pri],s.clock());
	//assert(queue_->pri_[pri].getLen(rdhold[pri]) > 0);
	  double timeout;
  	struct hdr_mac802_11n *mh;
	struct hdr_cmn *ch;
  	if (pktTx_[pri]){
		ch = HDR_CMN(pktTx_[pri]);
		if (ch->next_hop() != rdhold[pri]) {
		  pktTxHold_[pri] = pktTx_[pri];
		  pktTx_[pri] = 0;
		}
	} 
	if (!pktTx_[pri]) {
	      Packet* p = queue_->pri_[pri].deque(rdhold[pri]);
	      // framing
      		sendDATA(pri, p);
	}

      ch = HDR_CMN(pktTx_[pri]);
      printf(" uid:%d dst:%d\n",ch->uid(),ch->next_hop());
      mh = HDR_MAC802_11N(pktTx_[pri]);
      mh->dh_fc.fc_rdg = 0;

      rdhold[pri] = -1;

    // send
    if((u_int32_t)ETHER_ADDR(mh->dh_da) != MAC_BROADCAST){
      timeout = txtime(pktTx_[pri])
	                    + DSSS_EDCA_MaxPropagationDelay	
			    + sifs_
			    + DSSS_EDCA_MaxPropagationDelay
			    + txtime(getBAlen(),basicRate_);


    } else {
      timeout = txtime(pktTx_[pri]);
    }

    if (mhBackoff_.busy())
	mhBackoff_.stop();
    mhSifs_.start(pri, sifs_);
    //tx_resume();

}

// Add by Tomky : BA

int
Mac802_11n::sendBACK(int pri, Packet* pp)
{
	printf("node %d: sendBACK\n",index_);
        Packet *p = 0;
	/*
        while (p->aggr_){
           p->aggr_=p->aggr_->copy();
           p=p->aggr_;
        }*/
        p = Packet::alloc();
        hdr_cmn* ch = HDR_CMN(p);
        struct ack_frame *af = (struct ack_frame*)p->access(hdr_mac::offset_);
        //assert(pktCTRL_[pri] == 0);

	struct hdr_mac802_11n *dh = HDR_MAC802_11N(pp);
	u_int32_t dst = ETHER_ADDR(dh->dh_sa);

        ch->uid() = 0; // ACK-UID
        ch->ptype() = PT_MAC;
        ch->iface() = -2;
        ch->error() = 0;
        HDR_IP(p)->prio() = pri; //same priority as data packet
        bzero(af, MAC_HDR_LEN);

        af->af_fc.fc_protocol_version = MAC_ProtocolVersion;
        af->af_fc.fc_type       = MAC_Type_Control;
        af->af_fc.fc_subtype    = MAC_Subtype_BACK;
        af->af_fc.fc_to_ds      = 0;
        af->af_fc.fc_from_ds    = 0;
        af->af_fc.fc_more_frag  = 0;
        af->af_fc.fc_retry      = 0;
        af->af_fc.fc_pwr_mgt    = 0;
        af->af_fc.fc_more_data  = 0;
        af->af_fc.fc_wep        = 0;
        af->af_fc.fc_order      = 0;



	int n = 0;
	u_int64_t d = 0;
	u_int64_t k = 1;
	Packet *ppp = pp;
	Packet *pppp = 0;
	Packet *p_hold = 0; // Tomky : BA adv
	hdr_cmn* cch = HDR_CMN(pp);
	hdr_cmn* ch_hold = 0; // Tomky : BA adv
	
	// Tomky : BA adv
	if (macmib_.BACKType > 2) {
	  if (pktHold_[pri]) { // Hold a packet already, check if retransmission
	    ch_hold = HDR_CMN(pktHold_[pri]);
	    if (cch->uid() == ch_hold->uid()){
	      p_hold = pktHold_[pri]->aggr_;
	      ch_hold = HDR_CMN(p_hold);
	    } else { // Another packet is coming...Upload holded packet

	      ppp = pktHold_[pri]->aggr_;
	      pppp = ppp->aggr_;

              int tx = ch_hold->txtime();
              
	      pktHold_[pri]->aggr_ = 0;
	      Packet::free(pktHold_[pri]);
	      pktHold_[pri] = 0;
	      
	      while(ppp){
	        printf("node %d: Received other packet, forced to upload\n",index_);
                struct hdr_cmn* chh = HDR_CMN(ppp);
                if (!chh->error()){
                  printf("r");
                  chh->num_forwards() += 1;
                  chh->txtime() = tx;
                  chh->direction() = hdr_cmn::UP;
                  ppp->aggr_ = 0;
                  uptarget_->recv(ppp, (Handler*) 0);
                } else {
                  ppp->aggr_ = 0; // Fix AG
                  Packet::free(ppp);
                } 
                if (!pppp) break;
                ppp = pppp;
                pppp = ppp->aggr_;
              }
              printf("\n");
              
	    }
	  }
	  if (!pktHold_[pri]) {
	    printf("node %d: no holded packet!\n",index_);
	    pktHold_[pri] = pp->copy();
	    p_hold = pktHold_[pri];
	    while(p_hold->aggr_){
	      p_hold->aggr_ = p_hold->aggr_->copy();
	      p_hold = p_hold->aggr_;
	    }
	    p_hold = pktHold_[pri]->aggr_;
	    ch_hold = HDR_CMN(p_hold);
          }
        } 
        
	
	while (ppp->aggr_){
	        cch = HDR_CMN(ppp->aggr_);
	        
	        // Add by Tomky : BA adv
	        if (macmib_.BACKType > 2){
	        
	          if (p_hold){
	            ch_hold = HDR_CMN(p_hold);
                    while(p_hold &&  ch_hold->uid() != cch->uid() ){
                      //printf("%d  %d ",ch_hold->uid(),cch->uid());
                      p_hold = p_hold->aggr_;
                      if (p_hold) {
	                ch_hold = HDR_CMN(p_hold);
                      } 
                      //printf("%d\n",p_hold);
                    }
                    if (!p_hold) {
                      ch_hold = 0;
                    } 
                  
                    //printf("p_hold:%d ch_hold:%d\n",p_hold,ch_hold);
                    if (ch_hold) {
                    //  printf("%d - %d :",ch_hold->uid(),cch->uid());
                      ch_hold->error() = cch->error();
                    } else{
                      printf("node %d: DUP..\n",index_);
                      p_hold = pktHold_[pri]->aggr_;
                    }
                  }
                
                }
                
		if (!cch->error()) {
			d += k;
			printf("o");
			ppp = ppp->aggr_;
			
		} else {
			printf("x");
			//ppp = ppp->aggr_;
			pppp = ppp->aggr_->aggr_;
			ppp->aggr_->aggr_ = 0; // Fix AG
			Packet::free(ppp->aggr_);
			ppp->aggr_ = pppp;
		}
		k *= 2;
		n++;
	} 
	printf("\n");
	if (!d){
		//pp->aggr_ = 0;
		discard(pp,DROP_MAC_COLLISION);
		return 0;
	}

	
	// OK! So check if CTS packet is there...
	if (pktCTRL_[pri]){
                Packet::free(pktCTRL_[pri]); pktCTRL_[pri] = 0;
                mhSend_.stop();
	}

        if (macmib_.BACKType)
		ch->size() = phymib_.getBACKlen(macmib_.BACKType);

	PacketData* errInfo = new PacketData(8);
	u_int64_t data = d;
	for(int k = 0; k<8;k++){
		errInfo->data()[k] = data;
		data=data/256;
	}

	p->setdata(errInfo);

        //Add by Tomky : RD
        if((int)dst == rdhold[pri] && rdg_s == pri+1 && d==k-1) {
                af->af_fc.fc_rdg      = 1;
        } else {
                af->af_fc.fc_rdg      = 0;
		rdg_s = 0;
		rdhold[pri] = -1;
	}

        //af->af_duration = ACK_DURATION();
        STORE4BYTE(&dst, (af->af_ra));

        ch->txtime() = txtime(ch->size(), basicRate_);

        /* calculate ack duration */
        af->af_duration = 0;

        pktCTRL_[pri] = p;
        
        // Add by Tomky : BA adv
        if (macmib_.BACKType == 3) {
          
          Packet::free(pp->aggr_);
          pp->aggr_ = 0;
          ch = HDR_CMN(pp);
          ch->size() = 0;
          
          p_hold = pktHold_[pri]->aggr_;
          ch_hold = HDR_CMN(p_hold);
          
          while(!ch_hold->error()){
            printf("y");
            pp->aggr_ = p_hold;
            ch->size() += ch_hold->size();
            pktHold_[pri]->aggr_ = p_hold->aggr_;
            
            pp = pp->aggr_;
            p_hold = pktHold_[pri]->aggr_;
            
            if (!p_hold) break;
            
            ch_hold = HDR_CMN(p_hold);
          }
          pp->aggr_ = 0;
          printf("\n");
          
          if (!pktHold_[pri]->aggr_){
            Packet::free(pktHold_[pri]);
            pktHold_[pri] = 0;
          }
          
        } else if (macmib_.BACKType == 4){
          
          int size = 0;
          Packet::free(pp->aggr_);
          pp->aggr_ = 0;
          ch = HDR_CMN(pp);
          ch->size() = 0;
          
          p_hold = pktHold_[pri]->aggr_;
          ch_hold = HDR_CMN(p_hold);
          
          while(!ch_hold->error()){
            size += ch_hold->size();
            p_hold = p_hold->aggr_;
            
            if (!p_hold) {
              ch_hold = 0;
              break;
            }
            
            ch_hold = HDR_CMN(p_hold);
            
          }
          if (!ch_hold) {
            ch->size() = size;
            pp->aggr_ = pktHold_[pri]->aggr_;
            Packet::free(pktHold_[pri]);
            pktHold_[pri] = 0;
          } else {
            pp->aggr_ = 0;
          }
          
        
        }
//        printf("%d\n",d);
	return d;
}

// Add by Tomky : BA

void
Mac802_11n::recvBACK(Packet *p)
{

        int pri = LEVEL(p);

	// Add by Tomky : RD
	if (rdg_s == pri+1){
		printf("node %d: get ACK (RD)(BA), time: %f\n",index_,Scheduler::instance().clock());
		rdg_s = 5;
		rdhold[pri] = -1;
	}

        struct hdr_cmn *ch = HDR_CMN(p);
        if(tx_state_[pri] != MAC_SEND) {
            discard(p, DROP_MAC_INVALID_STATE);
        return;
        }
        assert(pktTx_[pri]->aggr_);

        //printf("(%d)...................recving ACK:%x\n",index_,p);
        //mhSend_.stop();
	//printf("here?\n");
	int all_ack = 1;
	u_int64_t b_ack = 0;
	for(int k = 7; k>=0;k--){
		b_ack *= 256;
		b_ack += p->accessdata()[k];
	}
	u_int64_t mod = 2;
	Packet *pp = pktTx_[pri]->aggr_;
	Packet *ppp = 0;
	while(pp->aggr_){
		pp=pp->aggr_;
		mod *= 2;
	}
	mod /= 2;
        ch = HDR_CMN(pktTx_[pri]);
        struct hdr_cmn *cch = 0;
        printf("node %d: b_ack:%d mod:%d\n",index_,b_ack,mod);
	pp = pktTx_[pri];
	while(mod){
		if (b_ack % 2) {
			cch = HDR_CMN(pp->aggr_);
			ch->size() -= cch->size();
			ppp = pp->aggr_;
			if (ppp) 
				{
				pp->aggr_ = pp->aggr_->aggr_;
				ppp->aggr_ = 0;
				Packet::free(ppp);
				}
			//ppp = 0;
			//pp = pp->aggr_;
			//printf("1");

		} else {
			//printf("0");
			all_ack = 0;
			pp = pp->aggr_;
		}
		b_ack = b_ack / 2;
		mod /= 2;
	}
        //printf("\n");


	if (!all_ack) {
		return;
	}
	printf("node %d: all ack!\n",index_);

	mhSend_.stop();
	ch = HDR_CMN(p);

        if(rd_right)
        {
        struct hdr_mac802_11n *mh = HDR_MAC802_11N(p);
                if(mh->dh_fc.fc_rdg == 1) {
                  rdg_r = pri+1;
                } else {
                  rdg_r = 0;
                }
                  printf("node %d: rdg=%d in ack\n",index_,mh->dh_fc.fc_rdg);
        }

        /*
         * The successful reception of this ACK packet implies
         * that our DATA transmission was successful.  Hence,
         * we can reset the Short/Long Retry Count and the CW.
         */
        if((u_int32_t) ch->size() <= macmib_.RTSThreshold)
                ssrc_[pri] = 0;
        else
                slrc_[pri] = 0;

        /* succesful transmission => give delay
         * to Akaroa
         */
        if(rtx_[pri]) rtx_[pri] = 0;
//        double delay=Scheduler::instance().clock() - start_handle_[pri];
        #if AKAROA > 0
        if(AKAROA index_ > 0) {
          AkObservation((6 * (index_ - 1)) + (pri + 4), delay);
          start_handle_[pri]=0;
        }
        #endif

        sending = 0;
        check_backoff_timer();
        /*
         * Backoff before sending again.
         */
	/*if(rdg_s == 5){
	  clean_aggr(pktTx_[pri]);
          Packet::free(pktTx_[pri]); pktTx_[pri] = 0;
	*/
        if(!cfb_ || ch->size() > (int)macmib_.RTSThreshold || rdg_s == 5) {
          //assert(mhBackoff_.backoff(pri) == 0); //Tomky : RD
          rst_cw(pri);
          mhBackoff_.start(pri, getCW(pri), is_idle());
          assert(pktTx_[pri]);
//         clean_aggr(pktTx_[pri]);

          Packet::free(pktTx_[pri]); pktTx_[pri] = 0;
          // Add by Tomky : RD
          if (pktTxHold_[pri]){
            pktTx_[pri] = pktTxHold_[pri];
            pktTxHold_[pri] = 0;
          }
	  BAlen = 0;
//          if (mhDefer_.busy()) mhDefer_.stop(); // Event UID fix?
          tx_resume();

        }
        else{
          // if this is the first packet in cfb, we must take its
          // duration into account, too.
          if(cfb_dur == 0) {
              cfb_dur = txtime(pktTx_[pri])
                            + sifs_ + txtime(getBAlen(),basicRate_);
          }

          if (cfb_dur && rd_right)
          if(rdg_r){
//                cfb_dur += txtime(pktTx_[pri]) + sifs_ + sifs_ + txtime(getBAlen(),basicRate_);

          }
	
          assert(pktTx_[pri]);


          Packet::free(pktTx_[pri]); pktTx_[pri] = 0;
	  BAlen = 0;

          cfb(pri);
        }
        mac_log(p);
}
