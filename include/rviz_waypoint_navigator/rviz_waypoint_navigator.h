/*
 * Copyright (c) 2017, Antoni Rosinol Vidal.
 * All rights reserved. *
 * Redistribution and use in source and binary forms, with or without
 * modification, are permitted provided that the following conditions are met:
 *
 *     * Redistributions of source code must retain the above copyright
 *       notice, this list of conditions and the following disclaimer.
 *     * Redistributions in binary form must reproduce the above copyright
 *       notice, this list of conditions and the following disclaimer in the
 *       documentation and/or other materials provided with the distribution.
 *     * The names of the contributors may not be used to endorse or promote
 *       products derived from this software without specific prior written
 *       permission.
 *
 * THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS "AS IS"
 * AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT LIMITED TO, THE
 * IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS FOR A PARTICULAR PURPOSE
 * ARE DISCLAIMED. IN NO EVENT SHALL THE COPYRIGHT OWNER OR CONTRIBUTORS BE
 * LIABLE FOR ANY DIRECT, INDIRECT, INCIDENTAL, SPECIAL, EXEMPLARY, OR
 * CONSEQUENTIAL DAMAGES (INCLUDING, BUT NOT LIMITED TO, PROCUREMENT OF
 * SUBSTITUTE GOODS OR SERVICES; LOSS OF USE, DATA, OR PROFITS; OR BUSINESS
 * INTERRUPTION) HOWEVER CAUSED AND ON ANY THEORY OF LIABILITY, WHETHER IN
 * CONTRACT, STRICT LIABILITY, OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE)
 * ARISING IN ANY WAY OUT OF THE USE OF THIS SOFTWARE, EVEN IF ADVISED OF THE
 * POSSIBILITY OF SUCH DAMAGE.
 */

#include <visualization_msgs/MarkerArray.h>
#include <interactive_markers/interactive_marker_server.h>
#include <interactive_markers/menu_handler.h>

// Forward declarations.
namespace tf {
  class Vector3;
}

void processFeedback(
    const visualization_msgs::InteractiveMarkerFeedbackConstPtr& feedback);

void alignMarker(
    const visualization_msgs::InteractiveMarkerFeedbackConstPtr &feedback);

void saveMarker(visualization_msgs::InteractiveMarker int_marker);

double rand(double min, double max);

//////////////////////////// Makers ////////////////////////////////////////////
visualization_msgs::Marker makeBox(
    const visualization_msgs::InteractiveMarker& msg);
visualization_msgs::InteractiveMarkerControl& makeBoxControl(visualization_msgs::InteractiveMarker& msg);
void makeRandomDofMarker(const tf::Vector3& position);
void makeViewFacingMarker(const tf::Vector3& position);
void makeQuadrocopterMarker(const tf::Vector3& position);
void makeChessPieceMarker(const tf::Vector3& position);
void makePanTiltMarker(const tf::Vector3& position);
void makeMenuMarker(const tf::Vector3& position);
void makeButtonMarker(const tf::Vector3& position);
void makeMovingMarker(const tf::Vector3& position);
void make6DofMarker(bool fixed, unsigned int interaction_mode,
                    const tf::Vector3& position, bool show_6dof);

