/*
 * Copyright (C) 2021 Open Source Robotics Foundation
 *
 * Licensed under the Apache License, Version 2.0 (the "License");
 * you may not use this file except in compliance with the License.
 * You may obtain a copy of the License at
 *
 *     http://www.apache.org/licenses/LICENSE-2.0
 *
 * Unless required by applicable law or agreed to in writing, software
 * distributed under the License is distributed on an "AS IS" BASIS,
 * WITHOUT WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied.
 * See the License for the specific language governing permissions and
 * limitations under the License.
 *
*/

#include "GzSceneManager.hh"

#include <ignition/common/Profiler.hh>
#include <ignition/gui/Application.hh>
#include <ignition/gui/GuiEvents.hh>
#include <ignition/gui/MainWindow.hh>
#include <ignition/plugin/Register.hh>
#include <ignition/rendering/RenderingIface.hh>
#include <ignition/rendering/Scene.hh>

#include "ignition/gazebo/EntityComponentManager.hh"
#include "ignition/gazebo/Events.hh"
#include "ignition/gazebo/components/Name.hh"
#include "ignition/gazebo/components/World.hh"
#include "ignition/gazebo/rendering/RenderUtil.hh"

namespace ignition
{
namespace gazebo
{
inline namespace IGNITION_GAZEBO_VERSION_NAMESPACE {
  /// \brief Private data class for GzSceneManager
  class GzSceneManagerPrivate
  {
    /// \brief Update the 3D scene based on the latest state of the ECM.
    public: void OnRender();

    //// \brief Pointer to the rendering scene
    public: rendering::ScenePtr scene;

    /// \brief Event Manager
    public: EventManager *eventManager{nullptr};

    /// \brief Is the simulation running the GUI and server in the same process
    public: bool sameProcess{false};

    /// \brief did the first render event occur?
    public: bool emitFirstRender{false};

    /// \brief Track connection to "EnableSensors" Event
    public: ignition::common::ConnectionPtr enableSensorsConn;

    /// \brief Rendering utility
    public: RenderUtil renderUtil;
  };
}
}
}

using namespace ignition;
using namespace gazebo;

/////////////////////////////////////////////////
GzSceneManager::GzSceneManager()
  : GuiSystem(), dataPtr(std::make_unique<GzSceneManagerPrivate>())
{
}

/////////////////////////////////////////////////
GzSceneManager::~GzSceneManager() = default;

/////////////////////////////////////////////////
void GzSceneManager::Configure(EventManager &_eventMgr, bool _sameProcess)
{
  if (this->dataPtr->eventManager)
  {
    ignerr << "Already have event manager, configure was called multiple times."
           << std::endl;
    return;
  }

  this->dataPtr->eventManager = &_eventMgr;
  this->dataPtr->sameProcess = _sameProcess;

  this->dataPtr->renderUtil.SetEventManager(_eventMgr);
}

/////////////////////////////////////////////////
void GzSceneManager::LoadConfig(const tinyxml2::XMLElement *)
{
  if (this->title.empty())
    this->title = "Scene Manager";

  ignition::gui::App()->findChild<
      ignition::gui::MainWindow *>()->installEventFilter(this);
}

//////////////////////////////////////////////////
void GzSceneManager::Update(const UpdateInfo &_info,
    EntityComponentManager &_ecm)
{
  IGN_PROFILE("GzSceneManager::Update");

  if (this->dataPtr->emitFirstRender && !this->dataPtr->sameProcess)
  {
    igndbg << "UpdateECM" << std::endl;
    this->dataPtr->renderUtil.UpdateECM(_info, _ecm);
    this->dataPtr->renderUtil.UpdateFromECM(_info, _ecm);
  }
}

/////////////////////////////////////////////////
bool GzSceneManager::eventFilter(QObject *_obj, QEvent *_event)
{
  if (_event->type() == gui::events::Render::kType)
  {
    this->dataPtr->OnRender();
    this->dataPtr->emitFirstRender = true;
  }

  // Standard event processing
  return QObject::eventFilter(_obj, _event);
}

/////////////////////////////////////////////////
void GzSceneManagerPrivate::OnRender()
{
  if (nullptr == this->scene)
  {
    this->scene = rendering::sceneFromFirstRenderEngine();
    if (nullptr == this->scene)
      return;

    this->renderUtil.SetScene(this->scene);
  }

  if (this->emitFirstRender && !this->sameProcess)
  {
    this->renderUtil.Update();
  }
}

// Register this plugin
IGNITION_ADD_PLUGIN(ignition::gazebo::GzSceneManager,
                    ignition::gui::Plugin)
