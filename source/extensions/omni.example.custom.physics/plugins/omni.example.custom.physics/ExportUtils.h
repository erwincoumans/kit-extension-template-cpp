// SPDX-FileCopyrightText: Copyright (c) 2023 NVIDIA CORPORATION & AFFILIATES. All rights reserved.
// SPDX-License-Identifier: Apache-2.0
//
// Licensed under the Apache License, Version 2.0 (the "License");
// you may not use this file except in compliance with the License.
// You may obtain a copy of the License at
//
// http://www.apache.org/licenses/LICENSE-2.0
//
// Unless required by applicable law or agreed to in writing, software
// distributed under the License is distributed on an "AS IS" BASIS,
// WITHOUT WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied.
// See the License for the specific language governing permissions and
// limitations under the License.

#pragma once

#include "PhysicsUtils.h"
#include <iostream>
#include <string>
#include <tinyxml2.h>

namespace omni {
namespace example {
namespace custom {
namespace physics {

class ExportUtils {
  public:
    pxr::UsdStageWeakPtr stage;
    UsdData &usdData;
    double metersPerUnit = 1.0;
    std::string baseDirPath;
    bool isLoaded = false;
    ExportUtils(pxr::UsdStageWeakPtr stage, UsdData &data);
    ~ExportUtils();
    void ExportToUrdf(const std::string fullPath);
    void ExportToMjcf(const std::string fullPath);
    void addUrdfInertial(tinyxml2::XMLDocument &xmlDoc, tinyxml2::XMLElement *parent, const pxr::SdfPath &childPath);
    void addUrdfLink(tinyxml2::XMLDocument &xmlDoc, tinyxml2::XMLElement *parent, const pxr::SdfPath &childPath);
    void addUrdfCollision(tinyxml2::XMLDocument &xmlDoc, tinyxml2::XMLElement *parent, const pxr::SdfPath &childPath);
    void addUrdfVisual(tinyxml2::XMLDocument &xmlDoc, tinyxml2::XMLElement *parent, const pxr::SdfPath &childPath);
    void shapeDescToUrdfXml(tinyxml2::XMLDocument &xmlDoc, tinyxml2::XMLElement *xmlElement,
                            const omni::physics::schema::ShapeDesc *shapeDesc);
    void shapeDescToMjcfXml(tinyxml2::XMLDocument &xmlDoc, tinyxml2::XMLElement *xmlElement,
                            const pxr::SdfPath &childPath, const omni::physics::schema::ShapeDesc *shapeDesc);
    tinyxml2::XMLElement *addMjcfLink(tinyxml2::XMLDocument &xmlDoc, tinyxml2::XMLElement *parent,
                                      const pxr::SdfPath &childPath, const pxr::SdfPath &parentPath = pxr::SdfPath());
    void addMjcfShapes(tinyxml2::XMLDocument &xmlDoc, tinyxml2::XMLElement *parent, const pxr::SdfPath &childPath);
    void addMjcfJoint(tinyxml2::XMLDocument &xmlDoc, tinyxml2::XMLElement *xmlElement, const pxr::SdfPath &primPath,
                      const pxr::SdfPath &parentPath);
    void buildKinematicTree(tinyxml2::XMLDocument &xmlDoc, tinyxml2::XMLElement *pRoot);
};

} // namespace physics
} // namespace custom
} // namespace example
} // namespace omni
