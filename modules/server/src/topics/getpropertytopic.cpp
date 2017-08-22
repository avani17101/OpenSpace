/*****************************************************************************************
 *                                                                                       *
 * OpenSpace                                                                             *
 *                                                                                       *
 * Copyright (c) 2014-2017                                                               *
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

#include <modules/server/include/jsonconverters.h>
#include <openspace/properties/property.h>
#include <openspace/scene/scene.h>
#include <modules/server/include/getpropertytopic.h>

using nlohmann::json;

namespace {
const char* _loggerCat = "GetPropertyTopic";
const char* AllPropertiesValue = "__allProperties";
const char* AllNodesValue = "__allNodes";
const char* PropertyKey = "property";
}

namespace openspace {

GetPropertyTopic::GetPropertyTopic()
        : Topic() {}

bool GetPropertyTopic::isDone() {
    return true;
}

void GetPropertyTopic::handleJson(json j) {
    std::string requestedKey = j.at(PropertyKey).get<std::string>();
    LDEBUG("Getting property '" + requestedKey + "'...");
    json response;
    if (requestedKey == AllPropertiesValue) {
        response = getAllProperties();
    }
    else if (requestedKey == AllNodesValue) {
        response = wrappedPayload(sceneGraph()->allSceneGraphNodes());
    }
    else {
        response = getPropertyFromKey(requestedKey);
    }
    _connection->sendJson(response);
}

json GetPropertyTopic::getAllProperties() {
    json jsonProps = json::array();
    for (const auto &prop : allProperties()) {
        jsonProps.push_back(prop);
    }
    json payload{{ "value", jsonProps }};
    return wrappedPayload(payload);
}

json GetPropertyTopic::getPropertyFromKey(const std::string& key) {
    properties::Property* prop = property(key);
    if (prop != nullptr) {
        json propJson = json::parse(prop->toJson());
        return wrappedPayload(propJson);
    }

    return wrappedError(fmt::format("property '{}' not found", key), 404);
}

} // namespace openspace
