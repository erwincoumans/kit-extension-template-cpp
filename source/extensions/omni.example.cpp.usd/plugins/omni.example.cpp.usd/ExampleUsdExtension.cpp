// Copyright (c) 2022, NVIDIA CORPORATION. All rights reserved.
//
// NVIDIA CORPORATION and its licensors retain all intellectual property
// and proprietary rights in and to this software, related documentation
// and any modifications thereto.  Any use, reproduction, disclosure or
// distribution of this software and related documentation without an express
// license agreement from NVIDIA CORPORATION is strictly prohibited.
//

#define CARB_EXPORTS

#include <carb/PluginUtils.h>

#include <omni/example/cpp/usd/IExampleUsdInterface.h>
#include <omni/ext/ExtensionsUtils.h>
#include <omni/ext/IExt.h>
#include <omni/kit/IApp.h>
#include <omni/timeline/ITimeline.h>
#include <omni/timeline/TimelineTypes.h>

#include <pxr/usd/usd/notice.h>
#include <pxr/usd/usd/stage.h>
#include <pxr/usd/usd/stageCache.h>
#include <pxr/usd/usd/primRange.h>
#include <pxr/usd/usdGeom/metrics.h>
#include <pxr/usd/usdGeom/xform.h>
#include <pxr/usd/usdUtils/stageCache.h>

#include <usdrt/scenegraph/usd/usd/stage.h>
#include <usdrt/scenegraph/usd/usd/prim.h>
#include <usdrt/scenegraph/usd/rt/xformable.h>
#include <usdrt/scenegraph/usd/sdf/path.h>
#include <usdrt/scenegraph/base/gf/vec3d.h>
#include <usdrt/scenegraph/base/gf/quatf.h>

#include <vector>

const struct carb::PluginImplDesc pluginImplDesc = { "omni.example.cpp.usd.plugin",
                                                     "An example C++ USDRT extension.", "NVIDIA",
                                                     carb::PluginHotReload::eEnabled, "dev" };

namespace omni
{
namespace example
{
namespace cpp
{
namespace usd
{

class ExampleCppUsdExtension : public IExampleUsdInterface
                             , public PXR_NS::TfWeakBase
{
protected:
    void createPrims() override
    {
        // It is important that all USD stage reads/writes happen from the main thread:
        // https ://graphics.pixar.com/usd/release/api/_usd__page__multi_threading.html
        if (!m_stage)
        {
            return;
        }
    }

    void removePrims() override
    {
        if (!m_stage)
        {
            return;
        }

        // Release all event subscriptions.
        PXR_NS::TfNotice::Revoke(m_usdNoticeListenerKey);
        m_timelineEventsSubscription = nullptr;
        m_updateEventsSubscription = nullptr;
    }

    void printStageInfo() const override
    {
        if (!m_stage)
        {
            return;
        }

        printf("---Stage Info Begin---\n");

        // Print the USD stage's up-axis.
        const PXR_NS::TfToken stageUpAxis = PXR_NS::UsdGeomGetStageUpAxis(m_stage);
        printf("Stage up-axis is: %s.\n", stageUpAxis.GetText());

        // Print the USD stage's meters per unit.
        const double metersPerUnit = PXR_NS::UsdGeomGetStageMetersPerUnit(m_stage);
        printf("Stage meters per unit: %f.\n", metersPerUnit);

        // Print the USD stage's prims.
        const PXR_NS::UsdPrimRange primRange = m_stage->Traverse();
        for (const PXR_NS::UsdPrim& prim : primRange)
        {
            printf("Stage contains prim: %s.\n", prim.GetPath().GetString().c_str());
        }

        printf("---Stage Info End---\n\n");
    }

    void startTimelineAnimation() override
    {
        if (omni::timeline::ITimeline* timeline = omni::timeline::getTimeline())
        {
            timeline->play();
        }
    }

    void stopTimelineAnimation() override
    {
        if (omni::timeline::ITimeline* timeline = omni::timeline::getTimeline())
        {
            timeline->stop();
        }
    }

    void onDefaultUsdStageChanged(long stageId) override
    {
        PXR_NS::TfNotice::Revoke(m_usdNoticeListenerKey);
        m_timelineEventsSubscription = nullptr;
        m_updateEventsSubscription = nullptr;
        m_primsWithRotationOps.clear();
        m_stage.Reset();

        if (stageId)
        {
            m_stage = PXR_NS::UsdUtilsStageCache::Get().Find(PXR_NS::UsdStageCache::Id::FromLongInt(stageId));
            m_rtStage = usdrt::UsdStage::Attach({static_cast<uint64_t>(stageId)});
            m_lastUpdateIndex = 0;
            m_usdNoticeListenerKey = PXR_NS::TfNotice::Register(PXR_NS::TfCreateWeakPtr(this), &ExampleCppUsdExtension::onObjectsChanged);

            if (m_instanceGeoPaths.empty())
            {
                const std::vector<std::string> assetTypes = {
                    "Cone", "Cube", "Disk", "Sphere", "Torus"
                };

                for (size_t i=0; i<250000; ++i)
                {
                    const int factorySector = (int)floor((float)i/1000.0f);
                    std::string pathStr = "/World/Factory_";
                    pathStr += std::to_string(factorySector);
                    pathStr += "/Asset_";
                    pathStr += std::to_string(i);
                    pathStr += "/";
                    pathStr += assetTypes[i%5];
                    const usdrt::SdfPath path(pathStr);
                    m_instanceGeoPaths.push_back(path);
                }
            }

            // Do all prefetch up front
            m_instanceGeoPrims.clear();
            m_instanceGeoPrims.reserve(250000);
            for (const usdrt::SdfPath& path: m_instanceGeoPaths)
            {
                m_instanceGeoPrims.push_back(m_rtStage->GetPrimAtPath(path));
            }

            std::cout << "prefetched all prims" << std::endl;

            // Subscribe to timeline events so we know when to start or stop animating the prims.
            if (auto timeline = omni::timeline::getTimeline())
            {
                m_timelineEventsSubscription = carb::events::createSubscriptionToPop(
                    timeline->getTimelineEventStream(),
                    [this](carb::events::IEvent* timelineEvent) {
                    onTimelineEvent(static_cast<omni::timeline::TimelineEventType>(timelineEvent->type));
                });
            }
        }
    }

    void onObjectsChanged(const PXR_NS::UsdNotice::ObjectsChanged& objectsChanged)
    {

    }

    void onTimelineEvent(omni::timeline::TimelineEventType timelineEventType)
    {
        switch (timelineEventType)
        {
        case omni::timeline::TimelineEventType::ePlay:
        {
            startAnimatingPrims();
        }
        break;
        case omni::timeline::TimelineEventType::eStop:
        {
            stopAnimatingPrims();
        }
        break;
        default:
        {

        }
        break;
        }
    }

    void startAnimatingPrims()
    {
        if (m_updateEventsSubscription)
        {
            // We're already animating the prims.
            return;
        }

        std::cout << "start animating prims" << std::endl;

        // Subscribe to update events so we can animate the prims.
        if (omni::kit::IApp* app = carb::getCachedInterface<omni::kit::IApp>())
        {
            m_updateEventsSubscription = carb::events::createSubscriptionToPop(app->getUpdateEventStream(), [this](carb::events::IEvent*)
            {
                onUpdateEvent();
            });
        }
    }

    void stopAnimatingPrims()
    {
        std::cout << "stop animating prims" << std::endl;

        m_updateEventsSubscription = nullptr;
        onUpdateEvent(); // Reset positions.
    }

    void onUpdateEvent()
    {
        // It is important that all USD stage reads/writes happen from the main thread:
        // https ://graphics.pixar.com/usd/release/api/_usd__page__multi_threading.html
        if (!m_stage)
        {
            return;
        }

        for (size_t i = 0; i < 10000; ++i)
        {
            const size_t index = (m_lastUpdateIndex + i) % 250000;
            usdrt::UsdPrim prim = m_instanceGeoPrims[index];
            usdrt::RtXformable xformable(prim);
            usdrt::UsdAttribute attr = xformable.GetWorldPositionAttr();
            usdrt::GfVec3d pos;
            attr.Get(&pos);
            pos[1] = (double)((std::rand()%200) - 100);
            attr.Set(pos);
            xformable.GetWorldOrientationAttr().Set(usdrt::GfQuatf(1));
        }

        m_lastUpdateIndex = (m_lastUpdateIndex + 10000) % 250000;
    }

private:
    struct PrimWithRotationOps
    {
        PXR_NS::UsdPrim m_prim;
        PXR_NS::UsdGeomXformOp m_localRotationOp;
        PXR_NS::UsdGeomXformOp m_globalRotationOp;
        bool m_invalid = false;
    };

    PXR_NS::UsdStageRefPtr m_stage;
    usdrt::UsdStageRefPtr m_rtStage;
    PXR_NS::TfNotice::Key m_usdNoticeListenerKey;
    size_t m_lastUpdateIndex;
    std::vector<PrimWithRotationOps> m_primsWithRotationOps;
    std::vector<usdrt::SdfPath> m_instanceGeoPaths;
    std::vector<usdrt::UsdPrim> m_instanceGeoPrims;
    carb::events::ISubscriptionPtr m_updateEventsSubscription;
    carb::events::ISubscriptionPtr m_timelineEventsSubscription;
};

}
}
}
}

CARB_PLUGIN_IMPL(pluginImplDesc, omni::example::cpp::usd::ExampleCppUsdExtension)

void fillInterface(omni::example::cpp::usd::ExampleCppUsdExtension& iface)
{
}
