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
#include <sdf/Radar.hh>

#include <ignition/common/Console.hh>
#include <ignition/gazebo/components/GpuRadar.hh>
#include <ignition/gazebo/EntityComponentManager.hh>

#include "ComponentInspectorEditor.hh"
#include "Radar.hh"
#include "Types.hh"

using namespace ignition;
using namespace gazebo;

/////////////////////////////////////////////////
Radar::Radar(ComponentInspectorEditor *_inspector)
{
  _inspector->Context()->setContextProperty("RadarImpl", this);
  this->inspector = _inspector;

  ComponentCreator creator =
    [=](EntityComponentManager &_ecm, Entity _entity, QStandardItem *_item)
  {
    auto comp = _ecm.Component<components::GpuRadar>(_entity);
    if (nullptr == _item || nullptr == comp)
      return;
    const sdf::Radar *radar = comp->Data().RadarSensor();

    _item->setData(QString("Radar"),
        ComponentsModel::RoleNames().key("dataType"));
    _item->setData(QList({
      QVariant(radar->RadarNoise().Mean()),
      QVariant(radar->RadarNoise().BiasMean()),
      QVariant(radar->RadarNoise().StdDev()),
      QVariant(radar->RadarNoise().BiasStdDev()),
      QVariant(radar->RadarNoise().DynamicBiasStdDev()),
      QVariant(radar->RadarNoise().DynamicBiasCorrelationTime()),

      QVariant(radar->HorizontalScanSamples()),
      QVariant(radar->HorizontalScanResolution()),
      QVariant(radar->HorizontalScanMinAngle().Radian()),
      QVariant(radar->HorizontalScanMaxAngle().Radian()),

      QVariant(radar->VerticalScanSamples()),
      QVariant(radar->VerticalScanResolution()),
      QVariant(radar->VerticalScanMinAngle().Radian()),
      QVariant(radar->VerticalScanMaxAngle().Radian()),

      QVariant(radar->RangeMin()),
      QVariant(radar->RangeMax()),
      QVariant(radar->RangeResolution()),

    }), ComponentsModel::RoleNames().key("data"));
  };

  this->inspector->RegisterComponentCreator(
      components::GpuRadar::typeId, creator);
}

/////////////////////////////////////////////////
Q_INVOKABLE void Radar::OnRadarNoise(
    double _mean, double _meanBias, double _stdDev,
    double _stdDevBias, double _dynamicBiasStdDev,
    double _dynamicBiasCorrelationTime)
{
  ignition::gazebo::UpdateCallback cb =
      [=](EntityComponentManager &_ecm)
  {
    auto comp = _ecm.Component<components::GpuRadar>(
        this->inspector->GetEntity());
    if (comp)
    {
      sdf::Radar *radar = comp->Data().RadarSensor();
      if (radar)
      {
        sdf::Noise noise = radar->RadarNoise();

        setNoise(noise, _mean, _meanBias, _stdDev, _stdDevBias,
            _dynamicBiasStdDev, _dynamicBiasCorrelationTime);

        radar->SetRadarNoise(noise);
      }
      else
        ignerr << "Unable to get the radar noise data.\n";
    }
    else
    {
      ignerr << "Unable to get the radar component.\n";
    }
  };
  this->inspector->AddUpdateCallback(cb);
}

/////////////////////////////////////////////////
Q_INVOKABLE void Radar::OnRadarChange(
    double _rangeMin, double _rangeMax,
    double _rangeResolution, double _horizontalScanSamples,
    double _horizontalScanResolution,
    double _horizontalScanMinAngle,
    double _horizontalScanMaxAngle, double _verticalScanSamples,
    double _verticalScanResolution, double _verticalScanMinAngle,
    double _verticalScanMaxAngle)
{
  ignition::gazebo::UpdateCallback cb =
      [=](EntityComponentManager &_ecm)
  {
    auto comp = _ecm.Component<components::GpuRadar>(
        this->inspector->GetEntity());
    if (comp)
    {
      sdf::Radar *radar = comp->Data().RadarSensor();
      if (radar)
      {
        radar->SetRangeMin(_rangeMin);
        radar->SetRangeMax(_rangeMax);
        radar->SetRangeResolution(_rangeResolution);

        radar->SetHorizontalScanSamples(_horizontalScanSamples);
        radar->SetHorizontalScanResolution(_horizontalScanResolution);
        radar->SetHorizontalScanMinAngle(_horizontalScanMinAngle);
        radar->SetHorizontalScanMaxAngle(_horizontalScanMaxAngle);

        radar->SetVerticalScanSamples(_verticalScanSamples);
        radar->SetVerticalScanResolution(_verticalScanResolution);
        radar->SetVerticalScanMinAngle(_verticalScanMinAngle);
        radar->SetVerticalScanMaxAngle(_verticalScanMaxAngle);
      }
      else
        ignerr << "Unable to get the radar data.\n";
    }
    else
    {
      ignerr << "Unable to get the radar component.\n";
    }
  };
  this->inspector->AddUpdateCallback(cb);
}

