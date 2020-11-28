 	/* -*- Mode:C++; c-file-style:"gnu"; indent-tabs-mode:nil; -*- */
/*
 * Copyright (c) 2020 R. Olusegun Alli-Oke
 *
 * This program is free software; you can redistribute it and/or modify
 * it under the terms of the GNU General Public License version 2 as
 * published by the Free Software Foundation;
 *
 * This program is distributed in the hope that it will be useful,
 * but WITHOUT ANY WARRANTY; without even the implied warranty of
 * MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
 * GNU General Public License for more details.
 *
 * You should have received a copy of the GNU General Public License
 * along with this program; if not, write to the Free Software
 * Foundation, Inc., 59 Temple Place, Suite 330, Boston, MA  02111-1307  USA
 *
 * Author: R. Olusegun Alli-Oke <razak.alli-oke@elizadeuniversity.edu.ng>
 */

//qDisc size is same as internal queue size because qDisc has only one internal queue. So, "this->GetCurrentSize();" (via qDisc class) is equivalent to "this->GetInternalQueue (0)-> GetCurrentSize(); " (via Queuebase).



#include <fstream>                              //file streams
#include <iostream>                             //standard i/o streams
#include "ns3/log.h"
#include "ns3/enum.h"
#include "ns3/uinteger.h"
#include "ns3/double.h"
#include "ns3/simulator.h"
#include "ns3/abort.h"
#include "pid-queue-disc.h"                     //also for ->GetNetDeviceQueueInterface ()
#include "ns3/drop-tail-queue.h"
#include "ns3/net-device-queue-interface.h"     //for ->GetTxQueue
#include "ns3/trace-helper.h"
#include "ns3/pointer.h"


namespace ns3 
{

NS_LOG_COMPONENT_DEFINE ("PidQueueDisc");

NS_OBJECT_ENSURE_REGISTERED (PidQueueDisc);

TypeId PidQueueDisc::GetTypeId (void)
    {
      static TypeId tid = TypeId ("ns3::PidQueueDisc")
        .SetParent<QueueDisc> ()
        .SetGroupName ("TrafficControl")
        .AddConstructor<PidQueueDisc> ()
        .AddAttribute ("a",
                       "Value of a",
                       DoubleValue (1.0),
                       MakeDoubleAccessor (&PidQueueDisc::m_a),
                       MakeDoubleChecker<double> ())
        .AddAttribute ("b",
                       "Value of b",
                       DoubleValue (0.0),
                       MakeDoubleAccessor (&PidQueueDisc::m_b),
                       MakeDoubleChecker<double> ())
        .AddAttribute ("c",
                       "Value of c",
                       DoubleValue (0.0),
                       MakeDoubleAccessor (&PidQueueDisc::m_c),
                       MakeDoubleChecker<double> ())
        .AddAttribute ("d",
                       "Value of d",
                       DoubleValue (0.0),
                       MakeDoubleAccessor (&PidQueueDisc::m_d),
                       MakeDoubleChecker<double> ())
        .AddAttribute ("e",
                       "Value of e",
                       DoubleValue (0.0),
                       MakeDoubleAccessor (&PidQueueDisc::m_e),
                       MakeDoubleChecker<double> ())
        .AddAttribute ("u0",
                       "Value of u0",
                       DoubleValue (0.0),
                       MakeDoubleAccessor (&PidQueueDisc::m_u0),
                       MakeDoubleChecker<double> ())
        .AddAttribute ("N",
                       "Value of N",
                       UintegerValue (0),
                       MakeUintegerAccessor (&PidQueueDisc::m_N),
                       MakeUintegerChecker<uint32_t> ())
        .AddAttribute ("Sstep",
                       "Start time to step-change in desired queue-length",
                       TimeValue (Seconds (0.0)),
                       MakeTimeAccessor (&PidQueueDisc::m_sStep),
                       MakeTimeChecker ())
        .AddAttribute ("Supdate",
                       "Start time to calculate drop probability",
                       TimeValue (Seconds (0.0)),
                       MakeTimeAccessor (&PidQueueDisc::m_sUpdate),
                       MakeTimeChecker ())
        .AddAttribute ("Tupdate",
                       "Time period to calculate drop probability",
                       TimeValue (Seconds (0.0)),
                       MakeTimeAccessor (&PidQueueDisc::m_tUpdate),
                       MakeTimeChecker ())
        .AddAttribute ("MaxSize",
                       "The maximum number of packets accepted by this queue disc",
                       QueueSizeValue (QueueSize ("1p")),
                       MakeQueueSizeAccessor (&PidQueueDisc::m_queueLimit),
                       MakeQueueSizeChecker ())
        .AddAttribute ("QueueSizeReferenceEQ",
                       "Equilibrium queue size used to approximate Model",
                       QueueSizeValue (QueueSize ("1p")),
                       MakeQueueSizeAccessor (&PidQueueDisc::m_QsizeRefEQ),
                       MakeQueueSizeChecker ())
        .AddAttribute ("QueueSizeReferenceDQ",
                       "Desired queue size",
                       QueueSizeValue (QueueSize ("1p")),
                       MakeQueueSizeAccessor (&PidQueueDisc::m_QsizeRefDQ),
                       MakeQueueSizeChecker ())
      ;

      return tid;
    }



// ------------------------------------------------- KEY METHODS----------------------------------------------------------------------------------------
// -----------------------------------------------------------------------------------------------------------------------------------------------------



// KEY METHOD 0: CONSTRUCTOR ( step a)
PidQueueDisc::PidQueueDisc ()
 : QueueDisc (QueueDiscSizePolicy::SINGLE_INTERNAL_QUEUE)
    {
      NS_LOG_FUNCTION (this);
      m_uv = CreateObject<UniformRandomVariable> ();
      int64_t strmm = 12345; m_uv->SetStream (strmm);
    }

// KEY METHOD 1: Check Configuration (step b)
bool PidQueueDisc::CheckConfig (void)
    {
      NS_LOG_FUNCTION (this);
      
      if (GetNQueueDiscClasses () > 0)
        {
          NS_LOG_ERROR ("PidQueueDisc cannot have classes");
          return false;
        }
      if (GetNPacketFilters () > 0)
        {
          NS_LOG_ERROR ("PidQueueDisc cannot have packet filters");
          return false;
        }
      if (GetNInternalQueues () == 0)
        {
          AddInternalQueue (CreateObjectWithAttributes<DropTailQueue<QueueDiscItem> > ("MaxSize", QueueSizeValue (m_queueLimit)));
        }
      if (GetNInternalQueues () != 1)
        {
          NS_LOG_ERROR ("PidQueueDisc needs 1 internal queue");
          return false;
        }

      return true;
    }
   

// KEY METHOD 2: Intialize Parameters of PID (step c)
void PidQueueDisc::InitializeParams (void)
    {
      // Initially queue is empty so variables are initialize to zero.
      m_dropProb = -1;                  // ensures that there is no drop when in open loop
      m_dropProb1 = 0;                  //since it is linearized controller (ie du0)
      m_dropProb2 = 0;
      m_ErrQsize1 = 0; 
      m_ErrQsize2 = 0;
      m_QsizeRef = m_QsizeRefEQ;
      std::cout << "linearization equilibrium queue-length: " <<  m_QsizeRef.GetValue() << std::endl;
      m_rtrsEvent1 = Simulator::Schedule (m_sStep, &PidQueueDisc::QSizeRefUpdate, this);
      m_rtrsEvent2 = Simulator::Schedule (m_sUpdate, &PidQueueDisc::CalculateP, this);     //step-change before updating drop ratio
    }
   

// KEY METHOD 3: DoEnqueue (step d)
bool PidQueueDisc::DoEnqueue (Ptr<QueueDiscItem> item)
    {
      NS_LOG_FUNCTION (this << item);   
      QueueSize nQueued = this->GetCurrentSize();                             
      bool bv =DropEarly (item);
      
      if (nQueued == m_queueLimit)                                         
        {
          //Drops due to queue limit: reactive --> drops packet if qdisc-queue size exceeds queue limit
          DropBeforeEnqueue (item, FORCED_DROP);
          return false;
        }
      else if (bv)
        {  
          //Early probability drop: proactive --> calls the DropEarly submethod --> which uses m_dropProb from the CalculateP sub-method
          DropBeforeEnqueue (item, UNFORCED_DROP);
          return false;
        }
          //No Drop
      bool retval = GetInternalQueue (0)->Enqueue (item);

          //If Queue::Enqueue fails, QueueDisc::DropBeforeEnqueue is called by the internal queue because QueueDisc::AddInternalQueue sets the trace callback

      NS_LOG_LOGIC ("\t bytesInQueue  " << GetInternalQueue (0)->GetNBytes ());
      NS_LOG_LOGIC ("\t packetsInQueue  " << GetInternalQueue (0)->GetNPackets ());

      return retval;
    }
   

// KEY METHOD 4: DoDequeue (step e)
Ptr<QueueDiscItem> PidQueueDisc::DoDequeue ()
    {
      NS_LOG_FUNCTION (this);   

      if (GetInternalQueue (0)->IsEmpty ())
        {
          NS_LOG_LOGIC ("Queue empty");
          return 0;
        }

      Ptr<QueueDiscItem> item = GetInternalQueue (0)->Dequeue ();
      
      return item;
    }


// KEY MWTHOD 5: DoPeek (Omitted, optional)


// KEY METHOD 6: Dispose (step ?)
void PidQueueDisc::DoDispose (void)  
   {
     NS_LOG_FUNCTION (this);
     m_uv = 0;
     Simulator::Remove (m_rtrsEvent1);
     Simulator::Remove (m_rtrsEvent2);
     QueueDisc::DoDispose ();
   }


// KEY METHOD 7: DECONSTRUCTOR (step f)
PidQueueDisc::~PidQueueDisc ()
   {
     NS_LOG_FUNCTION (this);
   }



// ------------------------------------------------- SUB-METHODS----------------------------------------------------------------------------------------
// -----------------------------------------------------------------------------------------------------------------------------------------------------



//SUB-METHODS for the KEY METHODS  
   
bool PidQueueDisc::DropEarly (Ptr<QueueDiscItem> item)                          
    { 
      NS_LOG_FUNCTION (this << item);      
      double p = m_dropProb;
      double v =  m_uv->GetValue ();
         
      //DropEarly is bypassed if queue (disc) has less than a couple of packets.OMITTED.
      
      //DropEarly is bypassed if DropProbability p is less than the random variable v \in [0, 1]
      if (p < v)                                                      
        {
          //if (p < 0) { std::cout << "Drop: " << "\t" << Simulator::Now ().GetSeconds () << "\t" << p << std::endl;} //checking there is no drop in open-loop mode
          return false;  //dont drop packet
        }

      return true;       //drop packet
    }


void PidQueueDisc::CalculateP () 
    {
      NS_LOG_FUNCTION (this);
      double pp;
      //QueueSize nQueued = this->GetCurrentSize();    //current queue size (packets) of qdisc (internal) queue
            Ptr<NetDeviceQueueInterface> qti = this->GetNetDeviceQueueInterface () ;
            Ptr<NetDevice> ndd = qti->GetObject<NetDevice>(); 
            PointerValue ptrV; ndd->GetAttribute ("TxQueue", ptrV);
            Ptr<Queue<Packet>> txQueue = ptrV.Get<Queue<Packet>>();
      m_Qsize = txQueue->GetCurrentSize();             //current queue size (packets) of netdevice (external) queue
      m_ErrQsize0 = double(m_QsizeRef.GetValue()) - double(m_Qsize.GetValue()); 
     
      //PID ( tustin (integral term) + backward difference (differential term) )  ;  (see equation 15) 
      m_dropProb0 = (m_a * m_dropProb1)  + (m_b * m_dropProb2)  +  ( m_c * m_ErrQsize0 )  +  ( m_d * m_ErrQsize1 )  +  ( m_e * m_ErrQsize2 );     //unsaturated drop probability, m_dropProb0

      m_dropProb2 = m_dropProb1;                      // store previous unsaturated drop probability
      m_dropProb1 = m_dropProb0;                      // saturation comes afterwards of storing m_dropProb0 not before it, see simulink file "p9_cc"
      m_ErrQsize2 = m_ErrQsize1;
      m_ErrQsize1 = m_ErrQsize0;

      m_dropProb0 = m_u0 + m_dropProb0;               // (see equation 18)

      pp = (m_dropProb0 <= 0) ? 0 : m_dropProb0; m_dropProb = (pp <= 1) ? pp : 1;                                                                //saturated drop probability, m_dropProb  

      m_rtrsEvent2 = Simulator::Schedule (m_tUpdate, &PidQueueDisc::CalculateP, this);
      
    //preferably after scheduling an m_tUpdate of m_rtrsEvent2; worst case, we miss some data points, doesnt affect computation of control action m_dropProb
      AsciiTraceHelper asciiTraceHelper;  Ptr<OutputStreamWrapper> stream1b = asciiTraceHelper.CreateFileStream ("/media/sf_SharedFolder/nsplots/Jplots/dprobp.dat",std::ios::app); 
      *stream1b->GetStream () << Simulator::Now ().GetSeconds () << "\t" << m_dropProb0 << "\t" << m_dropProb << "\t" << m_uv->GetValue() << std::endl;
    }


void PidQueueDisc::QSizeRefUpdate () 
    {
     NS_LOG_FUNCTION (this);
     m_QsizeRef = m_QsizeRefDQ;
     std::cout << "desired set-point reference queue-length: " <<  m_QsizeRef.GetValue() << "\n" << std::endl;
    }


void PidQueueDisc::PWMUpdate () 
    {
     NS_LOG_FUNCTION (this);
    }





} //namespace ns3


