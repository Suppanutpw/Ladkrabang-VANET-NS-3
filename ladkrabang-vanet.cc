#include <stdio.h>
#include <iostream>
#include "ns3/core-module.h"
#include "ns3/wave-mac-helper.h"
#include "ns3/yans-wifi-helper.h"
#include "ns3/wave-bsm-helper.h"
#include "ns3/wifi-80211p-helper.h"
#include "ns3/aodv-module.h"
#include "ns3/ipv4-address-helper.h"
#include "ns3/ipv4-interface-container.h"
#include "ns3/ipv4-list-routing-helper.h"
#include "ns3/flow-monitor-module.h"
#include "ns3/internet-stack-helper.h"
#include "ns3/applications-module.h"
#include "ns3/netanim-module.h"
#include "ns3/ns2-mobility-helper.h"

using namespace ns3;
using namespace std;

NS_LOG_COMPONENT_DEFINE ("Vanet-Ladkrabang-Tester");

// input trace file and config node count here !!!
int nodeNum = 120; // node count (have to match with tcl file!!!)
string traceFile ("/Users/spw/Desktop/ns-allinone-3.34/vanet_ladkrabang/scenario-3-120/vanetmobility.tcl");

NodeContainer c; // node (car)
Ptr<Socket> b_source[255]; // max to 255 node cause of time to run

int b_port = 8; // boardcast port
int u_port = 9; // unicast port
double duration = 100.01; // time to simulation
DoubleValue txp = (20); // Transmit power (dB)
DoubleValue freq = DoubleValue(5.9e9); // 802.11p 5.9 GHz
string logFile ("vanetTrace-ladkrabang.log"); // plaintext flow file
string netanimFile ("Vanetanim-ladkrabang.xml"); // netanim file
string flowmonFile ("vanet-routing-ladkrabang.flowmon"); // flow monitor xml file
string phyMode ("OfdmRate6MbpsBW10MHz"); // modulation
string rate ("2048bps"); // data rate
uint32_t packetSize = 1000; // bytes
uint32_t numPackets = 3; // unicast packet count per round
int64_t streamIndex = 0; // for BSM

Ipv4InterfaceContainer interface;
WaveBsmHelper waveBsmHelper;
// Returns the cumulative BSM Packet Delivery Ratio (PDR)
// which is the percent of cumulative expected packets within range
// that are actually received.
vector <double> txSafetyRanges;
double txDist1 = 50.0;
double txDist2 = 100.0;
double txDist3 = 150.0;
double txDist4 = 200.0;
double txDist5 = 250.0;
double txDist6 = 300.0;
double txDist7 = 350.0;
double txDist8 = 400.0;
double txDist9 = 450.0;
double txDist10 = 500.0;

// Convert to time object
Time interPacketInterval = Seconds (1.0);
TypeId tid = TypeId::LookupByName ("ns3::UdpSocketFactory");
Ptr<UniformRandomVariable> var = CreateObject<UniformRandomVariable> ();

static void GenerateTraffic (Ptr<Socket> socket, uint32_t pktSize,
                             uint32_t pktCount, Time pktInterval )
{
  if (pktCount > 0)
    {
      socket->Send (Create<Packet> (pktSize));
      Simulator::Schedule (pktInterval, &GenerateTraffic,
                           socket, pktSize, pktCount - 1, pktInterval);
    }
  else
    {
      socket->Close ();
    }
}

static inline string
PrintReceivedRoutingPacket (Ptr<Socket> socket, Ptr<Packet> packet, Address srcAddress)
{
  ostringstream oss;

  oss << Simulator::Now ().As (Time::S) << " node " << socket->GetNode ()->GetId ();
  InetSocketAddress addr = InetSocketAddress::ConvertFrom (srcAddress);
  oss << " received one packet from " << addr.GetIpv4 ();

  return oss.str ();
}

void
ReceiveBoardcastPacket (Ptr<Socket> socket)
{
  Ptr<Packet> packet;
  Address srcAddress;
  while ((packet = socket->RecvFrom (srcAddress)))
    {
      // print Boardcast packet detail
      NS_LOG_UNCOND ("Boardcast: " + PrintReceivedRoutingPacket (socket, packet, srcAddress));
    }

  InetSocketAddress addr = InetSocketAddress::ConvertFrom (srcAddress);

  Ptr<Socket> source = Socket::CreateSocket (socket->GetNode(), tid);
  InetSocketAddress remote = InetSocketAddress (Ipv4Address (addr.GetIpv4 ()), u_port);
  source->Connect (remote);
  // response in the same time
  /* Simulator::ScheduleWithContext (socket->GetNode()->GetId(),
                                  Simulator::Now(), &GenerateTraffic,
                                  source, packetSize, numPackets, interPacketInterval); */
  // response in variance time 
  Simulator::ScheduleWithContext (socket->GetNode()->GetId(),
                                  Simulator::Now() + Seconds (var->GetValue (0.0, 1.0)), &GenerateTraffic,
                                  source, packetSize, numPackets, interPacketInterval);
}

void
ReceiveUnicastPacket (Ptr<Socket> socket)
{
  Ptr<Packet> packet;
  Address srcAddress;
  while ((packet = socket->RecvFrom (srcAddress)))
    {
      // print Unicast packet detail
      NS_LOG_UNCOND ("Unicast: " + PrintReceivedRoutingPacket (socket, packet, srcAddress));
    }
}

// Physical Drop Debuger
class WifiPhyStats : public Object
{
public:
  void PhyTxDrop (std::string context, Ptr<const Packet> packet)
  {
    NS_LOG_UNCOND ("PHY Tx Drop");
  }

  void PhyRxDrop (std::string context, Ptr<const Packet> packet, WifiPhyRxfailureReason reason)
  {
    NS_LOG_UNCOND ("PHY Rx Drop " << reason);
  }
};
Ptr<WifiPhyStats> wifiPhyStats;

int main (int argc, char *argv[]) {
  txSafetyRanges.resize (10, 0);
  txSafetyRanges[0] = txDist1;
  txSafetyRanges[1] = txDist2;
  txSafetyRanges[2] = txDist3;
  txSafetyRanges[3] = txDist4;
  txSafetyRanges[4] = txDist5;
  txSafetyRanges[5] = txDist6;
  txSafetyRanges[6] = txDist7;
  txSafetyRanges[7] = txDist8;
  txSafetyRanges[8] = txDist9;
  txSafetyRanges[9] = txDist10;

  // set random seed for fix environment
  SeedManager::SetSeed (10);
  SeedManager::SetRun (1);

  // Create Ns2MobilityHelper with the specified trace log file as parameter
  Ns2MobilityHelper ns2 = Ns2MobilityHelper (traceFile);

  // open network flow log file for output
  ofstream os;
  os.open (logFile.c_str ());

  // Create nodes (vehicle).
  c.Create (nodeNum);

  ns2.Install (); // configure movements for each node, while reading trace file

  // The below set of helpers will help us to put together the wifi NICs we want
  YansWifiPhyHelper wifiPhy;
  YansWifiChannelHelper wifiChannel = YansWifiChannelHelper::Default ();
  Ptr<YansWifiChannel> channel = wifiChannel.Create ();
  wifiChannel.SetPropagationDelay ("ns3::ConstantSpeedPropagationDelayModel");
  wifiChannel.AddPropagationLoss ("ns3::FriisPropagationLossModel", "Frequency", DoubleValue (freq)); // Frequency must match with standard
  wifiChannel.AddPropagationLoss ("ns3::NakagamiPropagationLossModel"); // fading loss
  wifiPhy.SetChannel (channel);

  // set wave bsm for 10 range
  waveBsmHelper.Install  (interface,
                         Seconds (duration),
                         packetSize,
                         Seconds (0.1),
                         // GPS accuracy (i.e, clock drift), in number of ns
                         40.0,
                         txSafetyRanges,
                         0,
                         // tx max delay before transmit, in ms
                         MilliSeconds (10));  
  streamIndex += waveBsmHelper.AssignStreams (c, streamIndex);
  WaveBsmHelper::GetNodesMoving().resize (nodeNum, 0);
  waveBsmHelper.GetWaveBsmStats()->SetLogging (1);

  // Set Tx Power (db - gain)
  wifiPhy.Set ("TxPowerStart",DoubleValue (txp));
  wifiPhy.Set ("TxPowerEnd", DoubleValue (txp));

  // Set MAC layer 2
  NqosWaveMacHelper wifi80211pMac = NqosWaveMacHelper::Default ();
  Wifi80211pHelper wifi80211p = Wifi80211pHelper::Default ();
  wifi80211p.SetRemoteStationManager ("ns3::ConstantRateWifiManager",
                                      "DataMode",StringValue (phyMode),
                                      "ControlMode",StringValue (phyMode));
  NetDeviceContainer devices = wifi80211p.Install (wifiPhy, wifi80211pMac, c);

  // Physical Drop Reason
  // Config::Connect ("/NodeList/*/DeviceList/*/$ns3::WifiNetDevice/Phy/PhyTxDrop", MakeCallback (&WifiPhyStats::PhyTxDrop, wifiPhyStats));
  // Config::Connect ("/NodeList/*/DeviceList/*/$ns3::WifiNetDevice/Phy/PhyRxDrop", MakeCallback (&WifiPhyStats::PhyRxDrop, wifiPhyStats)); 

  // Internet & Routing use AODV protocol
  AodvHelper aodv;
  Ipv4ListRoutingHelper list;
  list.Add (aodv, 100);

  InternetStackHelper internet;
  internet.SetRoutingHelper (list);
  internet.Install (c);

  Ipv4AddressHelper ipv4;
  ipv4.SetBase ("10.1.1.0", "255.255.255.0");
  interface = ipv4.Assign (devices);

  // Setup receive node for transmissions
  for (uint32_t i = 0; i < nodeNum; i++)
    {
      // Boardcast Receiver
      Ptr<Socket> sink = Socket::CreateSocket (c.Get (i), tid);
      InetSocketAddress local = InetSocketAddress (Ipv4Address::GetAny(), b_port);
      sink->Bind (local);
      sink->SetRecvCallback (MakeCallback (&ReceiveBoardcastPacket));

      // Boardcast Transmitter
      b_source[i] = Socket::CreateSocket (c.Get (i), tid);
      InetSocketAddress remote = InetSocketAddress (Ipv4Address ("255.255.255.255"), b_port);
      b_source[i]->SetAllowBroadcast (true);
      b_source[i]->Connect (remote);

      // Unicast Receiver
      sink = Socket::CreateSocket (c.Get (i), tid);
      local = InetSocketAddress (interface.GetAddress (i), u_port);
      sink->Bind (local);
      sink->SetRecvCallback (MakeCallback (&ReceiveUnicastPacket));
    }

  // Generate Traffic
  // for (int time = 0; time < duration; time++)
  //  {
    for (int nodeid = 0; nodeid < nodeNum; nodeid++)
      {
        // insert this for dulation problem
        Simulator::ScheduleWithContext (b_source[nodeid]->GetNode ()->GetId (),
                                    Seconds (var->GetValue (0.0, 1.0)), &GenerateTraffic,
                                    b_source[nodeid], packetSize, duration, interPacketInterval);
      }
  //  }
  Simulator::Stop (Seconds (duration));

  // Wireshark Monitor File
  // AsciiTraceHelper ascii;
  // wifiPhy.EnableAsciiAll (ascii.CreateFileStream ("udp-echo.tr"));
  // wifiPhy.EnablePcapAll ("udp-echo", false);

  // Netanim file generation
  AnimationInterface anim(netanimFile);
  anim.SetMaxPktsPerTraceFile(99999999999);

  // Install Flowmonitor all nodes (support unicast packet monitor only!!)
  FlowMonitorHelper flowmon;
  Ptr<FlowMonitor> monitor = flowmon.InstallAll();

  Simulator::Run ();

  ///////////////////////// Network Perfomance Calculation ///////////////////////
  uint32_t SentPackets = 0;
  uint32_t ReceivedPackets = 0;
  uint32_t LostPackets = 0;
  int j=0;
  float AvgThroughput = 0;
  Time Jitter;
  Time Delay;

  Ptr<Ipv4FlowClassifier> classifier = DynamicCast<Ipv4FlowClassifier> (flowmon.GetClassifier ());
  map<FlowId, FlowMonitor::FlowStats> stats = monitor->GetFlowStats ();

  for (map<FlowId, FlowMonitor::FlowStats>::const_iterator iter = stats.begin (); iter != stats.end (); ++iter)
    {
      Ipv4FlowClassifier::FiveTuple t = classifier->FindFlow (iter->first);

      os << "----Flow ID: " << iter->first << endl;
      os << "Src Addr " << t.sourceAddress << " Dst Addr "<< t.destinationAddress << endl;
      os << "Sent Packets = " << iter->second.txPackets << endl;
      os << "Received Packets = " << iter->second.rxPackets << endl;
      os << "Lost Packets = " << iter->second.txPackets-iter->second.rxPackets << endl;
      os << "Packet delivery ratio = " << (double) iter->second.rxPackets*100/iter->second.txPackets << "%" << endl;
      os << "Packet loss ratio = " << (double) (iter->second.txPackets-iter->second.rxPackets)*100/iter->second.txPackets << "%" << endl;
      os << "Delay = " << iter->second.delaySum << endl;
      os << "Jitter = " << iter->second.jitterSum << endl;
      os << "Throughput = " << iter->second.rxBytes * 8.0/(iter->second.timeLastRxPacket.GetSeconds()-iter->second.timeFirstTxPacket.GetSeconds())/1024<<" Kbps" << endl;

      SentPackets = SentPackets +(iter->second.txPackets);
      ReceivedPackets = ReceivedPackets + (iter->second.rxPackets);
      LostPackets = LostPackets + (iter->second.txPackets-iter->second.rxPackets);
      AvgThroughput = AvgThroughput + (iter->second.rxBytes * 8.0/(iter->second.timeLastRxPacket.GetSeconds()-iter->second.timeFirstTxPacket.GetSeconds())/1024);
      Delay = Delay + (iter->second.delaySum);
      Jitter = Jitter + (iter->second.jitterSum);

      j = j + 1;

    }

  AvgThroughput = AvgThroughput/j;
  cout << "--------Total Results of the simulation----------" << endl;
  cout << "Total sent packets = " << SentPackets << endl;
  cout << "Total Received Packets = " << ReceivedPackets << endl;
  cout << "Total Lost Packets = " << LostPackets << endl;
  cout << "Packet Loss ratio = " << (double) ((LostPackets*100)/SentPackets) << "%" << endl;
  cout << "Packet delivery ratio = " << (double) ((ReceivedPackets*100)/SentPackets) << "%" << endl;
  cout << "Average Throughput = " << AvgThroughput << "Kbps" << endl;
  cout << "End to End Delay = " << Delay << endl;
  cout << "End to End Jitter delay = " << Jitter << endl;
  cout << "Total Flow id " << j << endl;


  monitor->SerializeToXmlFile(flowmonFile, true, true);

  //////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////
  
  Simulator::Destroy ();

  return 0;
}
