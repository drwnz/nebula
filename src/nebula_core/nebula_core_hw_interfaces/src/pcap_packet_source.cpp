// Copyright 2024 TIER IV, Inc.
//
// Licensed under the Apache License, Version 2.0 (the "License");
// you may not use this file except in compliance with the License.
// You may obtain a copy of the License at
//
//     http://www.apache.org/licenses/LICENSE-2.0
//
// Unless required by applicable law or agreed to in writing, software
// distributed under the License is distributed on an "AS IS" BASIS,
// WITHOUT WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied.
// See the License for the specific language governing permissions and
// limitations under the License.

#include <nebula_core_hw_interfaces/pcap_packet_source.hpp>

#include <pcap.h>
#include <netinet/ip.h>
#include <netinet/udp.h>
#include <net/ethernet.h>
#include <arpa/inet.h>

#include <iostream>
#include <algorithm>

namespace nebula::drivers
{

PcapPacketSource::PcapPacketSource() {}

PcapPacketSource::~PcapPacketSource()
{
  stop();
}

void PcapPacketSource::open(const std::string & pcap_file)
{
  pcap_file_ = pcap_file;
}

void PcapPacketSource::set_packet_callback(SensorPacketCallback callback)
{
  callback_ = callback;
}

void PcapPacketSource::start()
{
  if (running_) return;
  running_ = true;
  thread_ = std::thread(&PcapPacketSource::run, this);
}

void PcapPacketSource::stop()
{
  running_ = false;
  if (thread_.joinable()) {
    thread_.join();
  }
}

void PcapPacketSource::run()
{
  char errbuf[PCAP_ERRBUF_SIZE];
  pcap_t * handle = pcap_open_offline(pcap_file_.c_str(), errbuf);
  if (!handle) {
    std::cerr << "Could not open PCAP file: " << errbuf << std::endl;
    running_ = false;
    return;
  }

  int link_type = pcap_datalink(handle);
  if (link_type != DLT_EN10MB) {
      std::cerr << "Unsupported PCAP link type: " << link_type << ". Only Ethernet supported." << std::endl;
      pcap_close(handle);
      running_ = false;
      return;
  }

  struct pcap_pkthdr * header;
  const u_char * pkt_data;

  while (running_ && pcap_next_ex(handle, &header, &pkt_data) >= 0) {
    if (header->caplen < sizeof(struct ether_header)) continue;

    struct ether_header * eth_hdr = (struct ether_header *)pkt_data;
    uint16_t eth_type = ntohs(eth_hdr->ether_type);
    size_t eth_hdr_len = sizeof(struct ether_header);

    if (eth_type == 0x8100) {
        if (header->caplen < eth_hdr_len + 4) continue;
        eth_type = ntohs(*(uint16_t *)(pkt_data + eth_hdr_len + 2));
        eth_hdr_len += 4;
    }

    if (eth_type != ETHERTYPE_IP) continue;
    if (header->caplen < eth_hdr_len + sizeof(struct ip)) continue;

    struct ip * ip_hdr = (struct ip *)(pkt_data + eth_hdr_len);
    size_t ip_hdr_len = ip_hdr->ip_hl * 4;
    if (header->caplen < eth_hdr_len + ip_hdr_len) continue;

    uint16_t ip_off = ntohs(ip_hdr->ip_off);
    uint16_t frag_offset = (ip_off & IP_OFFMASK) * 8;
    bool more_frags = (ip_off & IP_MF) != 0;
    
    char src_ip_str[INET_ADDRSTRLEN];
    char dst_ip_str[INET_ADDRSTRLEN];
    inet_ntop(AF_INET, &(ip_hdr->ip_src), src_ip_str, INET_ADDRSTRLEN);
    inet_ntop(AF_INET, &(ip_hdr->ip_dst), dst_ip_str, INET_ADDRSTRLEN);

    if (ip_hdr->ip_p == IPPROTO_UDP) {
        const u_char * ip_payload = pkt_data + eth_hdr_len + ip_hdr_len;
        size_t ip_payload_len = ntohs(ip_hdr->ip_len) - ip_hdr_len;
        
        // Truncated packet check
        if (eth_hdr_len + ip_hdr_len + ip_payload_len > header->caplen) {
            std::cerr << "Warning: PCAP packet truncated, skipping." << std::endl;
            continue;
        }

        ReassemblyKey key{ip_hdr->ip_src.s_addr, ip_hdr->ip_dst.s_addr, ntohs(ip_hdr->ip_id), ip_hdr->ip_p};

        if (frag_offset == 0) {
            if (ip_payload_len < sizeof(struct udphdr)) continue;
            struct udphdr * udp_hdr = (struct udphdr *)ip_payload;
            uint16_t src_port = ntohs(udp_hdr->uh_sport);
            uint16_t dst_port = ntohs(udp_hdr->uh_dport);
            uint16_t udp_len = ntohs(udp_hdr->uh_ulen);

            if (!more_frags) {
                // Not fragmented
                if (callback_) {
                    SensorPacket sp;
                    sp.transport = SensorTransportKind::Replay;
                    sp.timestamp_ns = static_cast<uint64_t>(header->ts.tv_sec) * 1e9 + header->ts.tv_usec * 1e3;
                    sp.source = {src_ip_str, src_port};
                    sp.destination = {dst_ip_str, dst_port};
                    sp.payload.assign(ip_payload + sizeof(struct udphdr), ip_payload + udp_len);
                    callback_(sp);
                }
            } else {
                // First fragment of many
                auto & ass = assemblies_[key];
                ass.src_ip = src_ip_str;
                ass.dst_ip = dst_ip_str;
                ass.src_port = src_port;
                ass.dst_port = dst_port;
                ass.timestamp_ns = static_cast<uint64_t>(header->ts.tv_sec) * 1e9 + header->ts.tv_usec * 1e3;
                ass.total_size = udp_len - sizeof(struct udphdr);
                ass.data.resize(ass.total_size);
                ass.received.resize(ass.total_size, false);
                
                size_t frag_data_len = ip_payload_len - sizeof(struct udphdr);
                std::copy(ip_payload + sizeof(struct udphdr), ip_payload + ip_payload_len, ass.data.begin());
                std::fill(ass.received.begin(), ass.received.begin() + frag_data_len, true);
            }
        } else {
            // Subsequent fragment
            if (assemblies_.count(key)) {
                auto & ass = assemblies_[key];
                size_t udp_data_offset = frag_offset - sizeof(struct udphdr);
                std::copy(ip_payload, ip_payload + ip_payload_len, ass.data.begin() + udp_data_offset);
                std::fill(ass.received.begin() + udp_data_offset, ass.received.begin() + udp_data_offset + ip_payload_len, true);
                if (!more_frags) ass.saw_last = true;

                if (ass.is_complete()) {
                    if (callback_) {
                        SensorPacket sp;
                        sp.transport = SensorTransportKind::Replay;
                        sp.timestamp_ns = ass.timestamp_ns;
                        sp.source = {ass.src_ip, ass.src_port};
                        sp.destination = {ass.dst_ip, ass.dst_port};
                        sp.payload = std::move(ass.data);
                        callback_(sp);
                    }
                    assemblies_.erase(key);
                }
            }
        }
    }
  }

  pcap_close(handle);
  running_ = false;
}

}  // namespace nebula::drivers
