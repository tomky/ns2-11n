/* -*-	Mode:C++; c-basic-offset:8; tab-width:8; indent-tabs-mode:t -*- 
 *
 * Copyright (c) 1996 Regents of the University of California.
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
 *	Engineering Group at Lawrence Berkeley Laboratory and the Daedalus
 *	research group at UC Berkeley.
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
 *
 * $Header: /nfs/jade/vint/CVSROOT/ns-2/mac/wireless-phy.cc,v 1.25 2005/08/22 05:08:33 tomh Exp $
 *
 * Ported from CMU/Monarch's code, nov'98 -Padma Haldar.
 * wireless-phy.cc
 *
 *
 *adapted to phy-mimo.cc
 */

#include <math.h>

#include <packet.h>

#include <mobilenode.h>
#include <wireless-phy.h>
#include <propagation.h>
#include <modulation.h>
#include <omni-antenna.h>
#include <phy-mimo.h>
#include <packet.h>
#include <ip.h>
#include <agent.h>
#include <trace.h>

#include <mac-802_11n.h>
#include <random.h>
#include "diffusion/diff_header.h"

#define MAX(a,b) (((a)<(b))?(b):(a))

#define MREP
//#define mytest

/* ======================================================================
   MIMO Interface
   ====================================================================== */
static class MIMOClass: public TclClass {
public:
        MIMOClass() : TclClass("Phy/WirelessPhy/MIMO") {}
        TclObject* create(int, const char*const*) {
                return (new MIMO);
        }
} class_MIMO;


MIMO::MIMO()
{
//	WirelessPhy::WirelessPhy();
        bind("Noise_", &Noise_); //opower add
	num_antenna = 4;

#ifdef MREP
	bind("CPThresh_", &CPThresh_);
	bind("CSThresh_", &CSThresh_);
	bind("RXThresh_", &RXThresh_);
	//bind("bandwidth_", &bandwidth_);
	bind("Pt_", &Pt_);
	bind("freq_", &freq_);
	bind("L_", &L_);
#endif

}

int
MIMO::command(int argc, const char*const* argv)
{
//	TclObject *obj;

	/* Delete by Tomky 
	//opower+
        if(argc == 4) {
          if (strcasecmp(argv[1], "setPhyCodeRate") == 0) { 
                        if((atof(argv[2])==1)&&(atof(argv[3])==2))
                          code_rate=1;
                        else if((atof(argv[2])==2)&&(atof(argv[3])==3))
                          code_rate=2;
                        else if((atof(argv[2])==3)&&(atof(argv[3])==4))
                          code_rate=3;
                        else
                          code_rate=4;
                        if(code_rate!=0)
                          return TCL_OK;
                        else{
                          printf("The code rate isn't correct!!");
                          return TCL_ERROR;
                        }
                }
        //+opower
	*/

	// Add by Tomky : MIMO
	if (argc == 3) {
	  	if (strcasecmp(argv[1], "PhyNumAntenna") == 0) {
			 num_antenna = atoi(argv[2]);
			 return TCL_OK;
		} else if (strcasecmp(argv[1], "PhyMIMOSystem") == 0) {
			//mimo_system = atoi(argv[2]);
			return TCL_OK;
		}
	}


	return WirelessPhy::command(argc,argv);
}
 
void 
MIMO::sendDown(Packet *p)
{
	WirelessPhy::sendDown(p);
}

int 
MIMO::sendUp(Packet *p)
{
        /*
         * Sanity Check
         */
//        assert(initialized());
	
        PacketStamp s;
        double Pr;
        int pkt_recvd = 0;

	hdr_mac802_11n *dh = HDR_MAC802_11N(p);
	hdr_cmn *ch = HDR_CMN(p);
	int src_na = 0; // Number of Antenna of Source Node
	int src_mimo = 0; // MIMO System of Source Node
	double src_cr = 0;   // Code Rate of Source Node
	double BER = 0;

        Pr = p->txinfo_.getTxPr();
	
	double SNR = 0;

        // if the node is in sleeping mode, drop the packet simply
        if (em()) {
                        if (Is_node_on()!= true){
                        pkt_recvd = 0;
                        goto DONE;
                        }

                        if (Is_sleeping()==true && (Is_node_on() == true)) {
                                pkt_recvd = 0;
                                goto DONE;
                        }

        }
        // if the energy goes to ZERO, drop the packet simply
        if (em()) {
                if (em()->energy() <= 0) {
                        pkt_recvd = 0;
                        goto DONE;
                }
        }

        if(propagation_) {
                s.stamp((MobileNode*)node(), ant_, 0, lambda_);
                Pr = propagation_->Pr(&p->txinfo_, &s, this);
                if (Pr < CSThresh_) {
                        pkt_recvd = 0;
                        goto DONE;
                }
                if (Pr < RXThresh_) {
                        /*
                         * We can detect, but not successfully receive
                         * this packet.
                         */
                        hdr_cmn *hdr = HDR_CMN(p);
                        hdr->error() = 1;
#if DEBUG > 3
                        printf("SM %f.9 _%d_ drop pkt from %d low POWER %e/%e\n",
                               Scheduler::instance().clock(), node()->index(),
                               p->txinfo_.getNode()->index(),
                               Pr,RXThresh);
#endif
                }

        }
        if(modulation_) {
                hdr_cmn *hdr = HDR_CMN(p);
                hdr->error() = modulation_->BitError(Pr);
        }

        SNR = 10*log(Pr/Noise_);
//	printf("SNR:%f\n",SNR);
	// Tomky : Implement MIMO BER here!!!
	src_na = dh->dh_na;
	src_mimo = dh->dh_mimo;
	src_cr = dh->dh_cr;

	switch(src_mimo){
	case 0: // No setting on MIMO System...
		BER = 0;
		break;
	case 1: // SM-STBC

		switch(src_na){
		case 4:
			if (num_antenna == 4){
				if (!src_cr) {
					if (SNR <= 4)
						BER = (0.35-0.25)*(4-SNR)/4+0.25;
					else if (SNR <= 8)
						BER = (0.25-0.07)*(8-SNR)/4+0.07;
                                        else if (SNR <= 12)
                                                BER = (0.07-0.02)*(12-SNR)/4+0.02;
                                        else if (SNR <= 16)
                                                BER = (0.02-0.0022)*(16-SNR)/4+0.0022;
                                        else if (SNR <= 20)
                                                BER = (0.0022-0.00005)*(20-SNR)/4+0.00005;
					else if (SNR < 24 )
						BER = 0.00005;
					else
						BER = 0;

				} else {
	        	                pkt_recvd = 0;
        	        	        goto DONE;
				}
			} else {
	                        pkt_recvd = 0;
        	                goto DONE;
			}
			break;
		default:
                        pkt_recvd = 0;
                        goto DONE;
			break;
		}

		break;
	case 2: // SM-STTC
		
                switch(src_na){
                case 4:
			if (num_antenna == 4){

                                if (!src_cr) {
                                        if (SNR < 4)
                                                BER = (0.35-0.2)*(4-SNR)/4+0.2;
                                        else if (SNR < 8)
                                                BER = (0.2-0.08)*(8-SNR)/4+0.08;
                                        else if (SNR < 12)
                                                BER = (0.08-0.0065)*(12-SNR)/4+0.0065;
                                        else if (SNR < 16)
                                                BER = (0.0065-0.0001)*(16-SNR)/4+0.0001;
                                        else if (SNR < 20)
                                                BER = (0.0001-0.0000075)*(20-SNR)/4+0.0000075;
                                        else if (SNR < 24)
                                                BER = 0.0000075;
					else 
						BER = 0;


                                } else {
		                        pkt_recvd = 0;
                		        goto DONE;
                                }

			} else {
		               pkt_recvd = 0;
		               goto DONE;

			}
                        break;
                default:
                        pkt_recvd = 0;
                        goto DONE;
                        break;

                }

		break;
	case 3: // V-BLAST

                switch(src_na){                
		case 4:
			if (num_antenna == 4){
                                if (!src_cr) {


                                        if (SNR < 4)
                                                BER = (0.3-0.29)*(4-SNR)/4+0.29;
                                        else if (SNR < 8)
                                                BER = (0.29-0.21)*(8-SNR)/4+0.21;
                                        else if (SNR < 12)
                                                BER = (0.21-0.1)*(12-SNR)/4+0.1;
                                        else if (SNR < 16)
                                                BER = (0.1-0.05)*(16-SNR)/4+0.05;
                                        else if (SNR < 20)
                                                BER = (0.05-0.025)*(20-SNR)/4+0.025;
                                        else if (SNR <= 24)
                                                BER = (0.025 - 0.006)*(24-SNR)/4+0.006;
                                        else
                                                BER = 0.006;

                                } else if(src_cr >=(1/2)){

                                } else {
        	        	        pkt_recvd = 0;
	        	                goto DONE;
                                }

			} else {
	                        pkt_recvd = 0;
        	                goto DONE;
			}
                        break;
		case 2:
			if (num_antenna == 4){

                                if (!src_cr) {

                                        if (SNR < 4)
                                                BER = (0.25-0.19)*(4-SNR)/4+0.19;
                                        else if (SNR < 8)
                                                BER = (0.19-0.1)*(8-SNR)/4+0.1;
                                        else if (SNR < 12)
                                                BER = (0.1-0.025)*(12-SNR)/4+0.025;
                                        else if (SNR < 16)
                                                BER = (0.025-0.0035)*(16-SNR)/4+0.0035;
                                        else if (SNR < 20)
                                                BER = (0.0035-0.00019)*(20-SNR)/4+0.00019;
					else if (SNR <= 24)
						BER = (0.00019 - 0.000007)*(24-SNR)/4+0.000007;
                                        else
                                                BER = 0.000007;

                                } else if(src_cr >=(2/3)){

                                } else {
	        	                pkt_recvd = 0;
        	        	        goto DONE;
                                }

			} else if (num_antenna == 3){

                                        if (SNR < 4)
                                                BER = (0.3-0.25)*(4-SNR)/4+0.25;
                                        else if (SNR < 8)
                                                BER = (0.25-0.13)*(8-SNR)/4+0.13;
                                        else if (SNR < 12)
                                                BER = (0.13-0.035)*(12-SNR)/4+0.035;
                                        else if (SNR < 16)
                                                BER = (0.035-0.01)*(16-SNR)/4+0.01;
                                        else if (SNR < 20)
                                                BER = (0.01-0.0025)*(20-SNR)/4+0.0025;
                                        else if (SNR <= 24)
                                                BER = (0.0025 - 0.0003)*(24-SNR)/4+0.0003;
                                        else
                                                BER = 0.0003;


                        } else if (num_antenna == 2){

                                        if (SNR < 4)
                                                BER = (0.3-0.29)*(4-SNR)/4+0.29;
                                        else if (SNR < 8)
                                                BER = (0.29-0.21)*(8-SNR)/4+0.21;
                                        else if (SNR < 12)
                                                BER = (0.21-0.1)*(12-SNR)/4+0.1;
                                        else if (SNR < 16)
                                                BER = (0.1-0.05)*(16-SNR)/4+0.05;
                                        else if (SNR < 20)
                                                BER = (0.05-0.025)*(20-SNR)/4+0.025;
                                        else if (SNR <= 24)
                                                BER = (0.025 - 0.006)*(24-SNR)/4+0.006;
                                        else
                                                BER = 0.006;


			} else {
        	                pkt_recvd = 0;
	                        goto DONE;
			}
			break;
                default:
                        pkt_recvd = 0;
                        goto DONE;
                        break;

                }

		break;
	case 4: // Hybrid SM-STBC

                switch(src_na){
                case 3:
			if (num_antenna == 4){
				if (!src_cr) {

                                        if (SNR < 4)
                                                BER = (0.25-0.19)*(4-SNR)/4+0.19;
                                        else if (SNR < 8)
                                                BER = (0.19-0.1)*(8-SNR)/4+0.1;
                                        else if (SNR < 12)
                                                BER = (0.1-0.025)*(12-SNR)/4+0.025;
                                        else if (SNR < 16)
                                                BER = (0.025-0.0035)*(16-SNR)/4+0.0035;
                                        else if (SNR < 20)
                                                BER = (0.0035-0.00019)*(20-SNR)/4+0.00019;
                                        else if (SNR <= 24)
                                                BER = (0.00019 - 0.000007)*(24-SNR)/4+0.000007;
                                        else
                                                BER = 0.000007;

				} else {
					pkt_recvd = 0;
					goto DONE;
				}
			} else if (num_antenna == 3){

                                if (!src_cr) {

                                        if (SNR < 4)
                                                BER = (0.3-0.22)*(4-SNR)/4+0.22;
                                        else if (SNR < 8)
                                                BER = (0.22-0.13)*(8-SNR)/4+0.13;
                                        else if (SNR < 12)
                                                BER = (0.13-0.04)*(12-SNR)/4+0.04;
                                        else if (SNR < 16)
                                                BER = (0.04-0.011)*(16-SNR)/4+0.011;
                                        else if (SNR < 20)
                                                BER = (0.011-0.001)*(20-SNR)/4+0.001;
                                        else if (SNR <= 24)
                                                BER = (0.001 - 0.00009)*(24-SNR)/4+0.00009;
                                        else 
                                                BER = 0.00009;
					

                                } else {
        		                pkt_recvd = 0;
	                	        goto DONE;
                                }

			} else {
        	                pkt_recvd = 0;
	                        goto DONE;
			}
                        break;
                default:
                        pkt_recvd = 0;
                        goto DONE;
                        break;

                }

		break;
	default:
                pkt_recvd = 0;
                goto DONE;

		break;
	}
	
// 	printf("%d:SNR:%f, BER:%f, %d packet is to %d\n",node()->nodeid(),SNR,BER,ch->uid(),ch->next_hop());
	ch = HDR_CMN(p);

	// collision! all packets are error!
	
	if (ch->error() == 1) {
	  BER = 1;
	  pkt_recvd = 0;
	  goto DONE;
	}

	// Tomky : Finished BER calculating, now start emulate Bit-ERR..
	
	if (ch->next_hop() == node()->nodeid()){ // For wireless only
		if (!p->aggr_){
        		double e = Random::uniform(1);
	                int size = ch->size();
			int mycounter1 = 0;
        	        double P_e = BER*size;
                	if (P_e > (e))
			ch->error() = 1;
		} else {
			Packet* pp = p->aggr_;
 			printf("%d ",ch->uid());
#ifdef mytest
			int mycounter1=0;
#endif
	                while(pp) {
		        	double e = Random::uniform(1);
                	        struct hdr_cmn* cch = HDR_CMN(pp);
                        	if (BER*cch->size() > (e) ){
	        	               cch->error() = 1;
				       ch->error() = 1;
  				       printf("-");
                        	} else {
 					printf("+");
        	                       cch->error() = 0;
                		}
			    pp = pp->aggr_;
#ifdef mytest
			    mycounter1 ++;
			    if (mycounter1 >= 40) 
			    {
				printf("\n\n\n  %d FULLLLLLLLLLLLLLLLLLLLL \n",mycounter1);
				break;
// 				pp = 0;
			    }
#endif
		        }
			printf("\n");
		}
//        printf("uid:%d  SNR:%f BER:%f Err:%d Dest:%d\n",ch->uid(),SNR,BER,ch->error(),ch->next_hop());

	}

	//if (ch->error()) printf("Error!!\n");		
        /*
         * The MAC layer must be notified of the packet reception
         * now - ie; when the first bit has been detected - so that
         * it can properly do Collision Avoidance / Detection.
         */
        pkt_recvd = 1;

DONE:
        p->txinfo_.getAntenna()->release();


        /* WILD HACK: The following two variables are a wild hack.
           They will go away in the next release...
           They're used by the mac-802_11 object to determine
           capture.  This will be moved into the net-if family of
           objects in the future. */
        p->txinfo_.RxPr = Pr;
        p->txinfo_.CPThresh = CPThresh_;

        /*
         * Decrease energy if packet successfully received
         */
        if(pkt_recvd && em()) {

                double rcvtime = hdr_cmn::access(p)->txtime();
                // no way to reach here if the energy level < 0

                double start_time = MAX(channel_idle_time_, NOW);
                double end_time = MAX(channel_idle_time_, NOW+rcvtime);
                double actual_rcvtime = end_time-start_time;

                if (start_time > update_energy_time_) {
                        em()->DecrIdleEnergy(start_time-update_energy_time_,
                                             P_idle_);
                        update_energy_time_ = start_time;
                }

                em()->DecrRcvEnergy(actual_rcvtime,Pr_consume_);
/*
  if (end_time > channel_idle_time_) {
  status_ = RECVING;  //opower e
  }
*/
                channel_idle_time_ = end_time;
                update_energy_time_ = end_time;

                status_ = IDLE;

                /*
                  hdr_diff *dfh = HDR_DIFF(p);
                  printf("Node %d receives (%d, %d, %d) energy %lf.\n",
                  node()->address(), dfh->sender_id.addr_,
                  dfh->sender_id.port_, dfh->pk_num, node()->energy());
                */

                // log node energy
                if (em()->energy() > 0) {
                ((MobileNode *)node_)->log_energy(1);
                }

                if (em()->energy() <= 0) {
                        // saying node died
                        em()->setenergy(0);
                        ((MobileNode*)node())->log_energy(0);
                }
        }
//        printf("pkt_recvd:%d error:%d\n",pkt_recvd,ch->error());
        return pkt_recvd;

}
