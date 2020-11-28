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


#ifndef PID_QUEUE_DISC_H
#define PID_QUEUE_DISC_H

#include "ns3/queue-disc.h"
#include "ns3/nstime.h"
#include "ns3/boolean.h"
#include "ns3/data-rate.h"
#include "ns3/timer.h"
#include "ns3/event-id.h"
#include "ns3/random-variable-stream.h"


namespace ns3 {

class TraceContainer;
class UniformRandomVariable;

/**
 * \ingroup traffic-control
 *
 * \brief Implements Pid Active Queue Management discipline
 */
class PidQueueDisc : public QueueDisc
{
public:
  /**
   * \brief Get the type ID.
   * \return the object TypeId
   */
  static TypeId GetTypeId (void);

  /**
   * \brief PidQueueDisc Constructor
   */
  PidQueueDisc ();

  /**
   * \brief PidQueueDisc Destructor
   */
  virtual ~PidQueueDisc ();


   // Reasons for dropping packets
  static constexpr const char* UNFORCED_DROP = "Unforced drop";  //!< Early probability drops: proactive
  static constexpr const char* FORCED_DROP = "Forced drop";      //!< Drops due to queue limit: reactive

protected:
  /**
   * \brief Dispose of the object
   */
  virtual void DoDispose (void);
 

private:
  // ** Variables supplied by user
  Time m_sUpdate;                                              //!< Start time to use CalculateP () method to compute drop ratio
  Time m_tUpdate;                                              //!< compute drop probability every m_tUpdate secs (i.e. 1/m_tUpdate is the sampling frequency of discrete PID controller)
  Time m_sStep;                                                //!< start step-change to desired queue-length from linearation equilibrium queue-length
  uint32_t m_N;                                                //!< PID's derivative term's filter coefficient
  QueueSize  m_queueLimit;                                     //!< maximum queue-size of qdisc (internal) queue
  QueueSize  m_QsizeRef;                                       //!< variable for storing m_QsizeRefEQ or m_QsizeRefDQ
  QueueSize  m_QsizeRefEQ;                                     //!< Linearization equilibrium queue size
  QueueSize  m_QsizeRefDQ;                                     //!< Desired set-point reference queue size
  double m_a;                                                  //!< a parameter to PID difference-equation controller
  double m_b;                                                  //!< b parameter to PID difference-equation controller
  double m_c;                                                  //!< c parameter to PID difference-equation controller
  double m_d;                                                  //!< d parameter to PID difference-equation controller
  double m_e;                                                  //!< e parameter to PID difference-equation controller
  double m_u0;                                                 //!< Linearization equilbrium packet-loss ratio
  
  
  // ** Variables maintained by PID  
  QueueSize  m_Qsize;                                          //!< current value of external queue size
  double m_dropProb;                                           //!< Variable used in calculation of drop probability
  double m_dropProb0;                                          //!< current-step current value of of drop probability
  double m_dropProb1;                                          //!< 1-step previous value of drop probability
  double m_dropProb2;                                          //!< 2-step previous value of drop probability
  double m_ErrQsize0;                                          //!< current-step current value of queue-size error
  double m_ErrQsize1;                                          //!< 1-step previous value of queue-size error
  double m_ErrQsize2;                                          //!< 2-step previous value of queue-size error
  EventId m_rtrsEvent1;                                        //!< Event used to start (periodioc) computation of drop probability 
  EventId m_rtrsEvent2;                                        //!< Event used to repeatedly-schedule (periodioc) computation of drop probability 
  Ptr<UniformRandomVariable> m_uv;                             //!< Rng stream, random variable for comparison with computed drop probability

  /**
   * \brief Check if a packet needs to be dropped due to probability drop
   * \param item queue item
   * \param qSize queue size
   * \returns 0 for no drop, 1 for drop
   */
  bool DropEarly (Ptr<QueueDiscItem> item);                          

  /**
   * Periodically update the drop probability based on the delay samples:
   * not only the current delay sample but also the trend where the delay
   * is going, up or down
   */
  void CalculateP ();
  virtual bool CheckConfig (void);
   /**
   * \brief Initialize the queue parameters.
   */
  virtual void InitializeParams (void);
  virtual Ptr<QueueDiscItem> DoDequeue (void);
  virtual bool DoEnqueue (Ptr<QueueDiscItem> item);
  virtual void QSizeRefUpdate ();
  virtual void PWMUpdate ();
 
  
 
  
};

};   // namespace ns3

#endif

