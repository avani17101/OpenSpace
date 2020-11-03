/*****************************************************************************************
 *                                                                                       *
 * OpenSpace                                                                             *
 *                                                                                       *
 * Copyright (c) 2014-2019                                                               *
 *                                                                                       *
 * Permission is hereby granted, free of charge, to any person obtaining a copy of this  *
 * software and associated documentation files (the "Software"), to deal in the Software *
 * without restriction, including without limitation the rights to use, copy, modify,    *
 * merge, publish, distribute, sublicense, and/or sell copies of the Software, and to    *
 * permit persons to whom the Software is furnished to do so, subject to the following   *
 * conditions:                                                                           *
 *                                                                                       *
 * The above copyright notice and this permission notice shall be included in all copies *
 * or substantial portions of the Software.                                              *
 *                                                                                       *
 * THE SOFTWARE IS PROVIDED "AS IS", WITHOUT WARRANTY OF ANY KIND, EXPRESS OR IMPLIED,   *
 * INCLUDING BUT NOT LIMITED TO THE WARRANTIES OF MERCHANTABILITY, FITNESS FOR A         *
 * PARTICULAR PURPOSE AND NONINFRINGEMENT. IN NO EVENT SHALL THE AUTHORS OR COPYRIGHT    *
 * HOLDERS BE LIABLE FOR ANY CLAIM, DAMAGES OR OTHER LIABILITY, WHETHER IN AN ACTION OF  *
 * CONTRACT, TORT OR OTHERWISE, ARISING FROM, OUT OF OR IN CONNECTION WITH THE SOFTWARE  *
 * OR THE USE OR OTHER DEALINGS IN THE SOFTWARE.                                         *
 ****************************************************************************************/

#include <modules/autonavigation/pathsegment.h>

#include <modules/autonavigation/autonavigationmodule.h>
#include <modules/autonavigation/avoidcollisioncurve.h>
#include <modules/autonavigation/helperfunctions.h>
#include <modules/autonavigation/pathcurves.h>
#include <modules/autonavigation/rotationinterpolator.h>
#include <modules/autonavigation/zoomoutoverviewcurve.h>
#include <openspace/engine/globals.h>
#include <openspace/engine/moduleengine.h>
#include <openspace/interaction/navigationhandler.h>
#include <openspace/scene/scenegraphnode.h>
#include <openspace/util/camera.h>
#include <ghoul/logging/logmanager.h>

namespace {
    constexpr const char* _loggerCat = "PathSegment";

    const double Epsilon = 1E-7;
} // namespace

namespace openspace::autonavigation {

PathSegment::PathSegment(Waypoint start, Waypoint end, CurveType type)
    : _start(start), _end(end), _curveType(type)
{
    initCurve();
    _prevPose = start.pose;
}

void PathSegment::setStart(Waypoint cs) {
    _start = std::move(cs);
    initCurve();
}

const Waypoint PathSegment::start() const { return _start; }

const Waypoint PathSegment::end() const { return _end; }

const double PathSegment::pathLength() const { return _curve->length(); }

// TODO: remove function for debugging
const std::vector<glm::dvec3> PathSegment::getControlPoints() const {
    return _curve->getPoints();
}

double PathSegment::currentSpeed() const {
    // @TODO (emmbr, 2020-11-03) Ideally the speed should also depend on the size of
    // the visible object

    const SceneGraphNode* anchor = global::navigationHandler->anchorNode();
    const glm::dvec3 anchorPosition = anchor->worldPosition();
    double distanceToCurrentAnchor = glm::distance(_prevPose.position, anchorPosition);
    distanceToCurrentAnchor -= anchor->boundingSphere();
    double speed = distanceToCurrentAnchor;

    const double dampenStartUpTime = 2.0;

    // Dampen speed in beginning of path
    if (_progressedTime < dampenStartUpTime) {
        speed *= ghoul::cubicEaseIn(_progressedTime / dampenStartUpTime);
    }

    // Dampen speed in end of path, based on distance to target node
    // (At the end it's difficult to use time to dampen the motion, since we do not know
    // exactly how long it will take to reach th target)
    const double isCloseThreshold = static_cast<double>(5.f * anchor->boundingSphere());
    const double hasArrivedDistance = pathLength() - isCloseThreshold;
    if (_traveledDistance > pathLength() - isCloseThreshold) {
        const double distanceToEndPos = glm::distance(_prevPose.position, _end.position());
        speed *= ghoul::quadraticEaseIn(distanceToEndPos / isCloseThreshold) + 0.1;
    }

    return speed;
}

CameraPose PathSegment::traversePath(double dt) {
    if (!_curve || !_rotationInterpolator) {
        // TODO: handle better (abort path somehow)
        return _start.pose;
    }

    auto module = global::moduleEngine->module<AutoNavigationModule>();
    const double speedScale = module->AutoNavigationHandler().speedScale();

    double displacement = currentSpeed() * speedScale * dt;
    _progressedTime += dt;
    _traveledDistance += displacement;

    //LINFO(fmt::format("Traveled path ratio: {}", _traveledDistance / pathLength()));

    CameraPose pose = interpolatedPose(_traveledDistance);
    _prevPose = pose;

    return pose;
}

std::string PathSegment::getCurrentAnchor() const {
    bool pastHalfway = (_traveledDistance / pathLength()) > 0.5;
    return (pastHalfway) ? _end.nodeDetails.identifier : _start.nodeDetails.identifier;
}

bool PathSegment::hasReachedEnd() const {
    return (_traveledDistance / pathLength()) >= 1.0;
}

CameraPose PathSegment::interpolatedPose(double distance) const {
    double u = distance / pathLength();
    CameraPose cs;
    cs.position = _curve->positionAt(u);
    cs.rotation = _rotationInterpolator->interpolate(u);
    return cs;
}

void PathSegment::initCurve() {
    _curve.reset();

    switch (_curveType)
    {
    case CurveType::AvoidCollision:
        _curve = std::make_unique<AvoidCollisionCurve>(_start, _end);
        _rotationInterpolator = std::make_unique<EasedSlerpInterpolator>(
            _start.rotation(),
            _end.rotation()
        );
        break;

    case CurveType::Linear:
        _curve = std::make_unique<LinearCurve>(_start, _end);
        _rotationInterpolator = std::make_unique<EasedSlerpInterpolator>(
            _start.rotation(),
            _end.rotation()
        );
        break;

    case CurveType::ZoomOutOverview:
        _curve = std::make_unique<ZoomOutOverviewCurve>(_start, _end);
        _rotationInterpolator = std::make_unique<LookAtInterpolator>(
            _start.rotation(),
            _end.rotation(),
            _start.node()->worldPosition(),
            _end.node()->worldPosition(),
            _curve.get()
        );
        break;

    default:
        LERROR("Could not create curve. Type does not exist!");
        return;
    }

    if (!_curve || !_rotationInterpolator) {
        LERROR("Curve type has not been properly initialized.");
        return;
    }
}

} // namespace openspace::autonavigation
