/* -*- Mode:C++; c-file-style:"gnu"; indent-tabs-mode:nil; -*- */
/*
 * Copyright (c) October 2021 : R. Olusegun Alli-Oke
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
 *
Author: R. Olusegun Alli-Oke <razkgb2012@gmail.com>, <razak.alli-oke@elizadeuniversity.edu.ng>

This codes (MainScript.cc PidQueueDisc.cc, PidQueueDisc.h) are NS3 implementations of the work in 
"On the validity of numerical simulations for control-theoretic AQM schemes in computer networks"

Cite as: R. Olusegun Alli-Oke, On the validity of numerical simulations for control-theoretic AQM 
schemes in computer networks, Mathematics and Computers in Simulation, submitted 2020. GitHub repository: https://github.com/DROA2020/AQM-Simualtions (Nov. 2020).
 *  
 *       
 */
// ===========================================================================
//                      Dumbbell Network Topology
//
//             _                                                        _
//            |                                                          |  
//            | R1 -------------+                     +-------------- L1 |
//    nLeafL  |                 |                     |                  |  nLeafR 
//    Senders | R2 -----(L)LeftRouter(R)------(L)RightRouter(R) ----- L2 |  Receivers
//            |                 |                     |                  |
//            | R3 -------------+                     +-------------- L3 |
//            |_                                                        _|
//          
//
// nLeaf must be equal to nLeafR *only* because of the way the TCP client is set up i.e. Source(i) sends TCP data to corresponding Sink(i).
//
// LeftNode ->       LeftRouter         ->         RightRouter        -> RightNode
// LeftLeaf -> LeftRouterL||LeftRouterR -> RightRouterL||RightRouterR -> RightLeaf
//        dLeft           ||         dRouter           ||          dRight
//
// e.g if nLeaf=nLeafR=2, then:  LeftLeaf (2 devices) --> LeftRouterL (2 devices) || LeftRouterR (1 device) --> RightRouterL (1 device) || RightRouterR (2 devices) --> RightL (2 devices)
// 
//
// 
// ===========================================================================

//A point-to-point dumbbell topology is built from the scratch with the use of only 1st-degree helpers. Specifically, this code outputs the queue-lengths in the internal (qdisc) queue and external (NetDevice) queue, the 
//....the drop-probability, and the congestion window size. Bottleneck is at the LeftRouterR.

// ===========================================================================




#include "ns3/core-module.h"
#include "ns3/network-module.h"
#include "ns3/internet-module.h"
#include "ns3/point-to-point-module.h"
#include "ns3/applications-module.h"
#include "ns3/point-to-point-layout-module.h"
#include "ns3/traffic-control-module.h"
#include "ns3/netanim-module.h"    			
#include "ns3/mobility-module.h"
#include "ns3/flow-monitor-module.h" 
#include "ns3/config-store-module.h" 
#include "ns3/rng-seed-manager.h"

#include <iostream>
#include <sstream>
#include <iomanip>
#include <map>

using namespace ns3;

NS_LOG_COMPONENT_DEFINE ("PidExample");

std::string cwPath = "/NodeList/*/$ns3::TcpL4Protocol/SocketList/0/CongestionWindow" ;   

//trace-sink for cwnd size 
void CwndChange (Ptr<OutputStreamWrapper> stream0, std::string context, uint32_t oldCwnd, uint32_t newCwnd)
{   
  size_t found = context.find("/", 9+1);  
  *stream0->GetStream ()  << context.substr (10,found-10) << "\t" << Simulator::Now ().GetSeconds () << "\t" << oldCwnd << "\t" << newCwnd << std::endl;      
}

//trace-sink for TxQueue ExtQueue (Queuelength, NetDevice)
static void PktInQueueEQ(Ptr<OutputStreamWrapper> stream1, unsigned int beforeEQ, unsigned int nowEQ ){*stream1->GetStream () << Simulator::Now ().GetSeconds () << "\t" << nowEQ << std::endl;}

//trace-sink for QueueDisc IQueue (Queuelength, Qdisc)
static void PktInQueueIQ(Ptr<OutputStreamWrapper> stream2, unsigned int beforeIQ, unsigned int nowIQ ){*stream2->GetStream () << Simulator::Now ().GetSeconds () << "\t" << nowIQ << std::endl;}

//dynamic trace connection
static void TraceFunc(Ptr<QueueDisc> qdiscPtr, Ptr<Queue<Packet>> extqPtr)
{
  AsciiTraceHelper asciiTraceHelper;
  Ptr<OutputStreamWrapper> stream0 = asciiTraceHelper.CreateFileStream ("nsplots/Jplots/cwndp.dat");
  Ptr<OutputStreamWrapper> stream1 = asciiTraceHelper.CreateFileStream ("nsplots/Jplots/qsizep.dat");
  Ptr<OutputStreamWrapper> stream2 = asciiTraceHelper.CreateFileStream ("nsplots/Jplots/qdsizep.dat");
  Config::Connect(cwPath, MakeBoundCallback (&CwndChange, stream0));
  extqPtr->TraceConnectWithoutContext( "PacketsInQueue", MakeBoundCallback(&PktInQueueEQ, stream1) );
  qdiscPtr->TraceConnectWithoutContext( "PacketsInQueue", MakeBoundCallback(&PktInQueueIQ, stream2) );
}

//function header for animation
void BoundingBox (double ulx, double uly, double lrx, double lry, uint32_t nnLeafL, uint32_t nnLeafR, NodeContainer nncLeft, NodeContainer nncRouter, NodeContainer nncRight);





//////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////




int main ()
{
        
    RngSeedManager::SetSeed(1);  // Changes seed; default is 1. Fix Seed at 1 and vary Run from 1 to 5.
    RngSeedManager::SetRun(1);   // Changes run number; default is 1. Vary from 1 to 5 for independent runs of same simulation

    LogComponentEnable ("PidExample", LOG_LEVEL_INFO);

    bool isSack = false;
    double cstart = 1;                  //secs   //clientApp (source apps) start time
    double tstart = cstart + 0.001;     //secs   //start tracing of congestion window and queue lengths 
    double sstart = cstart + 80;        //secs   //start step-change from linearization equilibrium point (q0) to desired reference queue-length (qref) 
    double pstart = cstart;             //secs   //start to compute drop probability ; set pstart = sstart for open-loop mode b4 step-change, and pstart = cstart for closed-loop mode b4 step-change 
    double cstop  = cstart + 200;       //secs   //clientApp stop time; runtime + cstart
    uint32_t tcpMSSize = 500;           //Bytes  //Maximum Segment Size (MSS) of TCP packets
    uint32_t maxAppBytes = 0;           //Bytes  //maximum bytes to be sent by source app //Default is 0 --> infinite
    

    uint16_t port = 8080;
    uint32_t maxQueuePackets = 800;     //maximum size of external queue (netdevice queue)
    uint32_t qDiscLimitPkts = 800;      //maximum size of intenal queue (qdisc queue)
    uint32_t QsizeRefPktsEQ = 175;      //linearization equilibrium point of queue-length (q0)
    uint32_t QsizeRefPktsDQ = 475;      //desired reference queue-length (qref); step-change = QsizeRefPktsDQ - QsizeRefPktsEQ


    uint32_t    nLeafR = 60, nLeafL = 60;
    
   
    
NS_LOG_INFO ("\nSETTING DEFAULT PARAMETERS\n"); 
                                                                    
    Config::SetDefault ("ns3::TcpSocketBase::Sack", BooleanValue (isSack));                 //default is true 
    Config::SetDefault ("ns3::TcpSocket::DelAckCount", UintegerValue (0));                  //default is 2 packets 
    Config::SetDefault ("ns3::TcpL4Protocol::SocketType", StringValue ("ns3::TcpNewReno")); //default is TcpNewReno
    Config::SetDefault ("ns3::TcpSocketBase::WindowScaling", BooleanValue (false));         //default is true
    Config::SetDefault ("ns3::TcpSocket::RcvBufSize", UintegerValue (65535));               //default is 1131072 bytes  //Sets the AWND
    Config::SetDefault ("ns3::TcpSocketBase::MaxWindowSize", UintegerValue (65535));        //default is  65536 bytes   //limits the RcvBufSize value
    Config::SetDefault ("ns3::TcpSocket::InitialCwnd", UintegerValue (1));                  //default is 1 packet size  //equilibrium point w0
    Config::SetDefault ("ns3::TcpSocket::SegmentSize", UintegerValue (tcpMSSize));          //default is 536 Bytes

    Config::SetDefault ("ns3::QueueBase::MaxSize", QueueSizeValue (QueueSize (QueueSizeUnit::PACKETS, maxQueuePackets)));
    Config::SetDefault ("ns3::PidQueueDisc::MaxSize", QueueSizeValue (QueueSize (QueueSizeUnit::PACKETS, qDiscLimitPkts))); 
    Config::SetDefault ("ns3::PidQueueDisc::QueueSizeReferenceEQ", QueueSizeValue (QueueSize (QueueSizeUnit::PACKETS, QsizeRefPktsEQ))); 
    Config::SetDefault ("ns3::PidQueueDisc::QueueSizeReferenceDQ", QueueSizeValue (QueueSize (QueueSizeUnit::PACKETS, QsizeRefPktsDQ)));

      //PID (see equation 15)
      double T, u0, a, b, c, d, e;
      T = 0.00625;  u0 = 0.0085;
      //a = 1; b = 0; c = -18.22*(0.000001); d = +18.16*(0.000001);  e = 0;  //Hollot2002, (see equation 16)
      //a = 1; b = 0; c = -35.28*(0.000001); d = +35.22*(0.000001);  e = 0;  //Ustebay2007, (see equation 16)
      a = 1.8290; b = -0.8290; c = -0.0002556; d = +0.0005043;  e = -0.0002487;  //Kahe2014, (see equation 16)

      std::cout << "\tDiscretePIDparameters: " << "\tu0 = " << u0 << "\t\ta = " << a << "\t\tb = " << b << "\t\tc = " << c << "\t\td = " << d << "\t\te = " << e << "\n" << std::endl;
      std::cout << "\tDifferenceEquationPID: " << "\t" << "uk  =  u0  +  ( a * u_{k-1} )  +  ( b * u_{k-2} )  +  ( c * e_{k} )  +  ( d * e_{k-1} )  +  ( e * e_{k-2} )" << "\n" << std::endl;

      /* The snippet here is implemented in the PidQueueDisc.cc file placed in "\\src\\traffic-control\\model" folder
      //PID ( tustin (integral term) + backward difference (differential term) )  ;  (see equation 15)
      //uk = (m_a * u_{k-1})  + (m_b * u_{k-2})  +  ( m_c * e_{k} )  +  ( m_d * e_{k-1} )  +  ( m_e * e_{k-2} );  
      //m_dropProb0 = (m_a * m_dropProb1)  + (m_b * m_dropProb2)  +  ( m_c * m_ErrQsize0 )  +  ( m_d * m_ErrQsize1 )  +  ( m_e * m_ErrQsize2 );       //unsaturated drop probability, m_dropProb0
      //m_dropProb2 = m_dropProb1;    m_dropProb1 = m_dropProb0;               // store current and previous unsaturated drop probabilities .....m_dropProb0 needs to be stored before adding m_u0, see below
      //m_ErrQsize2 = m_ErrQsize1;   m_ErrQsize1 = m_ErrQsize0;                           // store current and previous errors
      //m_dropProb0 = m_u0 + m_dropProb0;              							// (see equation 18);  m_u0 needs to be added, see Figure 5
      //pp = (m_dropProb0 <= 0) ? 0 : m_dropProb0; m_dropProb = (pp <= 1) ? pp : 1;                       //saturated drop probability, m_dropProb . Saturation comes after equation 18 and not before it,  see Figure 5 */
    

    Config::SetDefault ("ns3::PidQueueDisc::a", DoubleValue (a));
    Config::SetDefault ("ns3::PidQueueDisc::b", DoubleValue (b));
    Config::SetDefault ("ns3::PidQueueDisc::c", DoubleValue (c));
    Config::SetDefault ("ns3::PidQueueDisc::d", DoubleValue (d));
    Config::SetDefault ("ns3::PidQueueDisc::e", DoubleValue (e));
    Config::SetDefault ("ns3::PidQueueDisc::u0", DoubleValue (u0));                         //linearization equilibrium point of drop probability
    Config::SetDefault ("ns3::PidQueueDisc::Tupdate", TimeValue (Seconds (T)));             //compute drop probability every T secs. (1/T sampling frequency of discrete PID controller)
    Config::SetDefault ("ns3::PidQueueDisc::Supdate", TimeValue (Seconds (pstart)));        //start to compute drop ratio
    Config::SetDefault ("ns3::PidQueueDisc::Sstep", TimeValue (Seconds (sstart)));          //start step-change from linearization equilibrium point (q0) to desired reference queue-length (qref)


    //delete previous-simulation output-data files
    remove("nsplots/Jplots/cwndp.dat");
    remove("nsplots/Jplots/qsizep.dat");
    remove("nsplots/Jplots/qdsizep.dat");
    remove("nsplots/Jplots/dprobp.dat");


NS_LOG_INFO ("CREATING DUMBELL TOPOLOGY\n"); 

    
    NS_LOG_INFO ("\tcreating nodes\n");

    // Create nodes for leafs and 2 routers
        NodeContainer ncLeft; NodeContainer ncRouter;  NodeContainer ncRight;
        ncLeft.Create (nLeafL); ncRouter.Create (2); ncRight.Create (nLeafR);                // order of creation is very important vis-a-vis IP addressing and NodeList in cwnd trace.
        NodeContainer nodes = NodeContainer::GetGlobal();

    NS_LOG_INFO ("\tcreating devices and channels\n");

    // Create the point-to-point right-link
        PointToPointHelper p2pLeafLinkL;
        p2pLeafLinkL.SetQueue ("ns3::DropTailQueue");  // MaxSize is set by Config::SetDefault("ns3::QueueBase.....), see above.
        p2pLeafLinkL.SetDeviceAttribute    ("DataRate", StringValue ("30Mbps"));
        p2pLeafLinkL.SetChannelAttribute   ("Delay", StringValue ("1ms")); 
        NetDeviceContainer dLeft, devLeftLeaf, devLeftRouterL;
        for (uint32_t i = 0; i < nLeafL; ++i)
        {
        dLeft  = p2pLeafLinkL.Install(ncLeft.Get(i), ncRouter.Get(0));
        devLeftLeaf.Add(dLeft.Get(0));
        devLeftRouterL.Add(dLeft.Get(1));       
        } 

    // Create the point-to-point bottleneck-link
        PointToPointHelper bottleNeckLink;
        bottleNeckLink.SetQueue ("ns3::DropTailQueue");  // MaxSize is set by Config::SetDefault("ns3::QueueBase.....), see above.
        bottleNeckLink.SetDeviceAttribute  ("DataRate", StringValue ("15Mbps")); 
        bottleNeckLink.SetChannelAttribute ("Delay", StringValue ("98ms"));
        NetDeviceContainer dRouter =  bottleNeckLink.Install(ncRouter.Get(0), ncRouter.Get(1));
        NetDeviceContainer devLeftRouterR, devRightRouterL;  
        devLeftRouterR.Add(dRouter.Get(0)); 
        devRightRouterL.Add(dRouter.Get(1));

    // Create the point-to-point left-link 
        PointToPointHelper p2pLeafLinkR;
        p2pLeafLinkR.SetQueue ("ns3::DropTailQueue");  // MaxSize is set by Config::SetDefault("ns3::QueueBase.....), see above.
        p2pLeafLinkR.SetDeviceAttribute    ("DataRate", StringValue ("30Mbps"));
        p2pLeafLinkR.SetChannelAttribute   ("Delay", StringValue ("1ms")); 
        NetDeviceContainer dRight, devRightRouterR, devRightLeaf;
        for (uint32_t i = 0; i < nLeafR; ++i)
        {
        dRight = p2pLeafLinkR.Install(ncRouter.Get(1), ncRight.Get(i));
        devRightRouterR.Add(dRight.Get(0)); 
        devRightLeaf.Add(dRight.Get(1));     
        } 

    NS_LOG_INFO ("\tinstalling Stack");

    // Install Stack
        InternetStackHelper stack;                                                                 //the InternetStackHelper Install command requires Ptr< Node > as argument
        stack.Install(ncLeft);                                                                     //automatically aggregates a traffic control layer to every left nodes
        stack.Install(ncRouter);                                                                   //automatically aggregates a traffic control layer to every router nodes
        stack.Install(ncRight);                                                                    //automatically aggregates a traffic control layer to every right nodes
                                                                                                                                                                               
        TrafficControlHelper tchBottleneck;  QueueDiscContainer CON_qDiscsLeftRouterR;             //set default qdisc on left and right nodes before "assigning IP address"
        tchBottleneck.SetRootQueueDisc ("ns3::PidQueueDisc");                                      //default is "ns3::FqCoDelQueueDisc"                                    
        CON_qDiscsLeftRouterR = tchBottleneck.Install (devLeftRouterR.Get(0));                     //the TrafficControlHelper Install command requires Ptr< NetDevice as argument   
         //CON_qDiscsRightRouterR = tchBottleneck.Install (devRightRouterR.Get(0));                //calls PidQueueDisc constructor, which has an event auto-scheduled. So do not uncomment unless you plan to use, becoz if... 
                                                                                                   //... if you delete or remove afterward, the constructor-scheduled-event will attempt to access a null pointer giving error

    NS_LOG_INFO ("\tassigning IP Addresses\n");
   
    // Assign IP Addresses 
        Ipv4AddressHelper ipv4L; 
        ipv4L.SetBase ("10.1.1.0", "255.255.255.0","0.0.0.1");
        Ipv4AddressHelper ipv4RT;
        ipv4RT.SetBase ("10.2.1.0", "255.255.255.0","0.0.0.1");
        Ipv4AddressHelper ipv4R;
        ipv4R.SetBase ("10.3.1.0", "255.255.255.0","0.0.0.1");

            //assign addresses to connection between left-side leaves and router
            Ipv4InterfaceContainer ipcLeftLeaf, ipcLeftRouterL;
            for (uint32_t i = 0; i < nLeafL; ++i) 
            {
                NetDeviceContainer dLL; 
                dLL.Add(devLeftLeaf.Get(i)); dLL.Add(devLeftRouterL.Get(i));
                Ipv4InterfaceContainer ipcLeft = ipv4L.Assign(dLL);
                ipcLeftLeaf.Add(ipcLeft.Get(0));
                ipcLeftRouterL.Add(ipcLeft.Get(1));
                ipv4L.NewNetwork(); 
            }

            //assign addresses to connecting routers
                NetDeviceContainer dLR; 
                dLR.Add(devLeftRouterR.Get(0)); dLR.Add(devRightRouterL.Get(0));
                Ipv4InterfaceContainer ipcRouter = ipv4RT.Assign(dLR);
                Ipv4InterfaceContainer ipcLeftRouterR, ipcRightRouterL;
                ipcLeftRouterR.Add(ipcRouter.Get(0));
                ipcRightRouterL.Add(ipcRouter.Get(1));
                ipv4RT.NewNetwork();

            //assign addresses to connection between router and right-side leaves
            Ipv4InterfaceContainer ipcRightRouterR, ipcRightLeaf;
            for (uint32_t i = 0; i < nLeafR; ++i) 
            {
                NetDeviceContainer dRR; 
                dRR.Add(devRightRouterR.Get(i)); dRR.Add(devRightLeaf.Get(i));
                Ipv4InterfaceContainer ipcRight = ipv4R.Assign(dRR);
                ipcRightRouterR.Add(ipcRight.Get(0));
                ipcRightLeaf.Add(ipcRight.Get(1));  
                ipv4R.NewNetwork();
            }

    // Enable global routing
        Ipv4GlobalRoutingHelper::PopulateRoutingTables();

    // Removing default root queue disc (and associated filters, classes and queues) 
        for (uint32_t i = 0; i < nLeafL; ++i){tchBottleneck.Uninstall (ncLeft.Get(i)->GetDevice(0));}                 //removal from left-leaf devices
        for (uint32_t i = 0; i < nLeafR; ++i){tchBottleneck.Uninstall (ncRight.Get(i)->GetDevice(0));}                //removal from right-leaf devices
        tchBottleneck.Uninstall (devLeftRouterL);                                                                     //removal from left router - right
        tchBottleneck.Uninstall (devRightRouterL);                                                                    //removal from right router - left 
        tchBottleneck.Uninstall (devRightRouterR);                                                                    //removal from right router - right

    // Accessing pointers to LeftRouterR root queue disc and netdevice queue
        Ptr<TrafficControlLayer> tcl0 = ncRouter.Get(0)->GetObject<TrafficControlLayer> ();  
        Ptr<QueueDisc> qDiscsLeftRouterR = tcl0->GetRootQueueDiscOnDevice (devLeftRouterR.Get(0));
       
        Ptr<QueueDisc> IntQD = qDiscsLeftRouterR;                                                                     //qdisc pointer (==> internal queue) of LeftRouterR (see description)
        PointerValue ptrV; devLeftRouterR.Get(0)->GetAttribute ("TxQueue", ptrV);
        Ptr<Queue<Packet>> ExtQ = ptrV.Get<Queue<Packet>>();                                                          //netdevice (external queue) pointer of LeftRouterR (see description)

        
    NS_LOG_INFO ("\tinstalling Source Apps and Sink Apps\n");

    // Install TCP socket & Source App to the left-side leaf-nodes
        BulkSendHelper clientHelper ("ns3::TcpSocketFactory", Address());
        clientHelper.SetAttribute ("MaxBytes", UintegerValue (maxAppBytes));
        clientHelper.SetAttribute ("SendSize", UintegerValue (tcpMSSize));

        ApplicationContainer clientApps;
        for (uint32_t i = 0; i < nLeafL; ++i)
        {
          AddressValue remoteAddress (InetSocketAddress (ipcRightLeaf.GetAddress(i,0), port));
          clientHelper.SetAttribute ("Remote", remoteAddress);
          clientApps.Add (clientHelper.Install (ncLeft.Get(i)));
        }
        clientApps.Start(Seconds(cstart));                                                                           //start 1 second after sinkApp
        clientApps.Stop(Seconds(cstop));                                                                             //stop before the sinkApp stops 

    // Install TCP socket & Sink App to the right-side leaf-nodes
        Address sinkLocalAddress (InetSocketAddress (Ipv4Address::GetAny(), port));
        PacketSinkHelper packetSinkHelper ("ns3::TcpSocketFactory", sinkLocalAddress);

        ApplicationContainer sinkApps;
        for (uint32_t i = 0; i < nLeafR; ++i)
        {
        sinkApps.Add (packetSinkHelper.Install (ncRight.Get(i)));
        }
        sinkApps.Start(Seconds(0));                                                                                  //start immediately
        sinkApps.Stop(Seconds(cstop+1.0));                                                                           //stop 1 sec after sourceApp stops  
 
    /*NS_LOG_INFO ("\tinstalling Mobility and Toplogy Animation\n");

    // Install stationary mobility on the nodes, i.e. fixed nodes
        MobilityHelper mobility;
        mobility.SetMobilityModel ("ns3::ConstantPositionMobilityModel");
        mobility.Install(nodes); 

    // Using NetAnim animator        
        AnimationInterface anim ("nsplots/Jplots/allTraces/animV.xml");	
        BoundingBox (1, 1, 100, 100, nLeafL, nLeafR, ncLeft, ncRouter, ncRight);                                    //uncomment BoundingBox method	
        anim.SetMaxPktsPerTraceFile(99999999999999);	                                                            //prevents the warning of "Max Packets per trace file exceeded"        				
        //anim.EnablePacketMetadata (true);                                                                         //optional
        //anim.EnableIpv4L3ProtocolCounters (Seconds (0), Seconds (10));                                            //optional

    NS_LOG_INFO ("\tenabling PCap and FlowMonitor Tracing\n");

    // Additional pcap tracing
        PointToPointHelper helper;
        helper.EnablePcapAll ("nsplots/Jplots/allTraces/pcapT", false);
        FlowMonitorHelper flowmon;  Ptr<FlowMonitor> monitor = flowmon.InstallAll();


NS_LOG_INFO ("VERIFY CONFIGURATION DETAILS\n");

GtkConfigStore config;
config.ConfigureDefaults();
config.ConfigureAttributes();*/   


NS_LOG_INFO ("ENTERING SIMULATION PHASE\n"); 

//simulation phase
    std::cout << "***Running the simulation***\n" << std::endl;
    Simulator::Schedule (Seconds(tstart), &TraceFunc, IntQD, ExtQ);
    Simulator::Stop (Seconds(cstop+10.0));
    Simulator::Run ();

    //monitor->SerializeToXmlFile("nsplots/Jplots/allTraces/flowM.xml", true, true);         //comment this if above "Additional pcap tracing" section is commented

    std::cout << "***Stats from the bottleneck queue disc***" << std::endl;
  
    QueueDisc::Stats st = qDiscsLeftRouterR->GetStats ();
    std::cout << st << std::endl;
    
    if (st.GetNDroppedPackets (PidQueueDisc::UNFORCED_DROP) == 0)
    {
      std::cout << "There should be some unforced drops\n" << std::endl;
    }
    if (st.GetNDroppedPackets (QueueDisc::INTERNAL_QUEUE_DROP) != 0)
    {
      std::cout << "There should be zero drops due to queue full\n" << std::endl; 
    }

    std::cout << "***Destroying the simulation***\n" << std::endl;
    Simulator::Destroy ();

    Ptr<PacketSink> sink1 = DynamicCast<PacketSink> (sinkApps.Get (0));
    std::cout << "Total Bytes Received by Sink(i) from Source(i): " << sink1->GetTotalRx () << "\n" << std::endl;

    return 0;
}




//....................................................................................................................................................................
//....................................................................................................................................................................




/*
// Bounding Box for NetAnimator (Edited Code from ns3::PointToPointDumbbellHelper Class)
    void BoundingBox (double ulx, double uly, // Upper left x/y
                      double lrx, double lry, // Lower right x/y
                      uint32_t nnLeafL, uint32_t nnLeafR, NodeContainer nncLeft, NodeContainer nncRouter, NodeContainer nncRight)
    {

    uint32_t bx_nLeafL = nnLeafL;
    uint32_t bx_nLeafR = nnLeafR;
    NodeContainer bx_ncLeft = nncLeft; 
    NodeContainer bx_ncRight = nncRight;
    NodeContainer bx_ncRouter = nncRouter;
    
      double xDist;
      double yDist;
      if (lrx > ulx)
        {
          xDist = lrx - ulx;
        }
      else
        {
          xDist = ulx - lrx;
        }
      if (lry > uly)
        {
          yDist = lry - uly;
        }
      else
        {
          yDist = uly - lry;
        }

      double xAdder = xDist / 3.0;
      double  thetaL = M_PI / (bx_nLeafL + 1.0);
      double  thetaR = M_PI / (bx_nLeafR + 1.0);

      // Place the left router
      Ptr<Node> lr = bx_ncRouter.Get(0);
      Ptr<ConstantPositionMobilityModel> loc = lr->GetObject<ConstantPositionMobilityModel> ();
      if (loc == 0)
        {
          loc = CreateObject<ConstantPositionMobilityModel> ();
          lr->AggregateObject (loc);
        }
      Vector lrl (ulx + xAdder, uly + yDist/2.0, 0);
      loc->SetPosition (lrl);

      // Place the right router
      Ptr<Node> rr = bx_ncRouter.Get(1);
      loc = rr->GetObject<ConstantPositionMobilityModel> ();
      if (loc == 0)
        {
          loc = CreateObject<ConstantPositionMobilityModel> ();
          rr->AggregateObject (loc);
        }
      Vector rrl (ulx + xAdder * 2, uly + yDist/2.0, 0); // Right router location
      loc->SetPosition (rrl);

      // Place the left leaf nodes
      double theta = -M_PI_2 + thetaL;
      for (uint32_t l = 0; l < bx_nLeafL; ++l)
        {
          // Make them in a circular pattern to make all line lengths the same
          // Special case when theta = 0, to be sure we get a straight line
          if ((bx_nLeafL % 2) == 1)
            { // Count is odd, see if we are in middle
              if (l == (bx_nLeafL / 2))
                {
                  theta = 0.0;
                }
            }
          Ptr<Node> ln = bx_ncLeft.Get(l);
          loc = ln->GetObject<ConstantPositionMobilityModel> ();
          if (loc == 0)
            {
              loc = CreateObject<ConstantPositionMobilityModel> ();
              ln->AggregateObject (loc);
            }
          Vector lnl (lrl.x - std::cos (theta) * xAdder,
                      lrl.y + std::sin (theta) * xAdder, 0);   // Left Node Location
          // Insure did not exceed bounding box
          if (lnl.y < uly) 
            {
              lnl.y = uly; // Set to upper left y
            }
          if (lnl.y > lry) 
            {
              lnl.y = lry; // Set to lower right y
            }
          loc->SetPosition (lnl);
          theta += thetaL;
        }
      // Place the right nodes
      theta = -M_PI_2 + thetaR;
      for (uint32_t r = 0; r < bx_nLeafR; ++r)
        {
          // Special case when theta = 0, to be sure we get a straight line
          if ((bx_nLeafR % 2) == 1)
            { // Count is odd, see if we are in middle
              if (r == (bx_nLeafR / 2))
                {
                  theta = 0.0;
                }
            }
          Ptr<Node> rn = bx_ncRight.Get(r);
          loc = rn->GetObject<ConstantPositionMobilityModel> ();
          if (loc == 0)
            {
              loc = CreateObject<ConstantPositionMobilityModel> ();
              rn->AggregateObject (loc);
            }
          Vector rnl (rrl.x + std::cos (theta) * xAdder, // Right node location
                      rrl.y + std::sin (theta) * xAdder, 0);
          // Insure did not exceed bounding box
          if (rnl.y < uly) 
            {
              rnl.y = uly; // Set to upper left y
            }
          if (rnl.y > lry) 
            {
              rnl.y = lry; // Set to lower right y
            }
          loc->SetPosition (rnl);
          theta += thetaR;
        }
    }
*/


//........................................................................................THE END.....................................................................
//....................................................................................................................................................................


