/* -*-	Mode:C++; c-basic-offset:8; tab-width:8; indent-tabs-mode:t -*-  *
 *
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
 *
 * $Header: /nfs/jade/vint/CVSROOT/ns-2/mac/wireless-phy.h,v 1.14 2005/06/13 17:50:41 haldar Exp $
 *
 * Ported from CMU/Monarch's code, nov'98 -Padma Haldar.
 *
 * wireless-phy.h
 * -- a SharedMedia network interface
 *
 *
 *
 * adapted to phy-mimo.h
 */

#ifndef ns_MIMO_h
#define ns_MIMO_h


#include "propagation.h"
#include "modulation.h"
#include "omni-antenna.h"
#include "wireless-phy.h"
#include "mobilenode.h"
#include "timer-handler.h"

#define MREP

class Phy;
class Propagation;
class MIMO;


//
class MIMO : public WirelessPhy {
public:
	MIMO();
	
	virtual void sendDown(Packet *p);
	virtual int sendUp(Packet *p);
	
	inline double getNoise() {return Noise_; } //opower add
  
	virtual int command(int argc, const char*const* argv);

protected:
	double Noise_; // opower add
	double code_rate; // opower add
	int num_antenna; // Add by Tomky : MIMO
#ifdef MREP
	double RXThresh_;	// receive power threshold (W)
	double CSThresh_;	// carrier sense threshold (W)
	double CPThresh_;	// capture threshold (db)
	double freq_;           // frequency
	double L_;		// system loss factor
	double Pt_;		// transmitted signal power (W)
#endif

private:
        EnergyModel* em() { return node()->energy_model(); }
	friend class Sleep_Timer;

};

#endif /* !ns_MIMO_h */
