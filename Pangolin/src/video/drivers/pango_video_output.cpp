/* This file is part of the Pangolin Project.
 * http://github.com/stevenlovegrove/Pangolin
 *
 * Copyright (c) 2014 Steven Lovegrove
 *
 * Permission is hereby granted, free of charge, to any person
 * obtaining a copy of this software and associated documentation
 * files (the "Software"), to deal in the Software without
 * restriction, including without limitation the rights to use,
 * copy, modify, merge, publish, distribute, sublicense, and/or sell
 * copies of the Software, and to permit persons to whom the
 * Software is furnished to do so, subject to the following
 * conditions:
 *
 * The above copyright notice and this permission notice shall be
 * included in all copies or substantial portions of the Software.
 *
 * THE SOFTWARE IS PROVIDED "AS IS", WITHOUT WARRANTY OF ANY KIND,
 * EXPRESS OR IMPLIED, INCLUDING BUT NOT LIMITED TO THE WARRANTIES
 * OF MERCHANTABILITY, FITNESS FOR A PARTICULAR PURPOSE AND
 * NONINFRINGEMENT. IN NO EVENT SHALL THE AUTHORS OR COPYRIGHT
 * HOLDERS BE LIABLE FOR ANY CLAIM, DAMAGES OR OTHER LIABILITY,
 * WHETHER IN AN ACTION OF CONTRACT, TORT OR OTHERWISE, ARISING
 * FROM, OUT OF OR IN CONNECTION WITH THE SOFTWARE OR THE USE OR
 * OTHER DEALINGS IN THE SOFTWARE.
 */

#include <pangolin/video/drivers/pango_video_output.h>
#include <pangolin/utils/picojson.h>
#include <set>

namespace pangolin
{

const std::string pango_video_type = "raw_video";

PangoVideoOutput::PangoVideoOutput(const std::string& filename)
    : packetstream(filename), packetstreamsrcid(-1)
{
}

PangoVideoOutput::~PangoVideoOutput()
{
}

const std::vector<StreamInfo>& PangoVideoOutput::Streams() const
{
    return streams;
}

void PangoVideoOutput::SetStreams(const std::vector<StreamInfo>& st, const std::string& uri, const json::value& properties)
{
    std::set<unsigned char*> unique_ptrs;
    for(size_t i=0; i<st.size(); ++i) {
        unique_ptrs.insert(st[i].Offset());
    }

    if(unique_ptrs.size() < st.size()) {
        throw std::invalid_argument("Each image must have unique offset into buffer.");
    }

    if(packetstreamsrcid == -1) {
        input_uri = uri;
        streams = st;
        device_properties = properties;
    }else{
        throw std::runtime_error("Unable to add new streams");
    }
}

void PangoVideoOutput::WriteHeader()
{
    json::value json_header(json::object_type,false);
    json::value& json_streams = json_header["streams"];
    json_header["device"] = device_properties;

    total_frame_size = 0;
    for(unsigned int i=0; i< streams.size(); ++i) {
        StreamInfo& si = streams[i];
        total_frame_size += si.SizeBytes();

        json::value& json_stream = json_streams.push_back();
        json_stream["encoding"] = si.PixFormat().format;
        json_stream["width"] =    si.Width();
        json_stream["height"] =   si.Height();
        json_stream["pitch"] =    si.Pitch();
        json_stream["offset"] =   (size_t)si.Offset();
    }

    packetstreamsrcid = packetstream.AddSource(
        pango_video_type, input_uri, json_header,
        total_frame_size,
        "struct Frame{"
        " uint8 stream_data[" + pangolin::Convert<std::string,size_t>::Do(total_frame_size) + "];"
        "};"
    );
}

int PangoVideoOutput::WriteStreams(unsigned char* data, const json::value& frame_properties)
{
    if(packetstreamsrcid == -1) {
        WriteHeader();
    }

    if(!frame_properties.is<json::null>()) {
        packetstream.WriteSourcePacketMeta(packetstreamsrcid, frame_properties);
    }

    packetstream.WriteSourcePacket(
        packetstreamsrcid,
        (char*)data, total_frame_size
    );

    return 0;
}

}
