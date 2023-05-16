// Copyright (c) 2022, NVIDIA CORPORATION. All rights reserved.
//
// NVIDIA CORPORATION and its licensors retain all intellectual property
// and proprietary rights in and to this software, related documentation
// and any modifications thereto.  Any use, reproduction, disclosure or
// distribution of this software and related documentation without an express
// license agreement from NVIDIA CORPORATION is strictly prohibited.
//

#define CARB_EXPORTS

#include <mujoco/mujoco.h>
#define MAX_MJ_ERROR_LENGTH 500

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

#include <vector>

const struct carb::PluginImplDesc pluginImplDesc = { "omni.example.cpp.usd.plugin",
                                                     "An example C++ extension.", "NVIDIA",
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
        mjv_defaultScene(&m_scn);

        char error[MAX_MJ_ERROR_LENGTH] = "";
        //m_m = mj_loadXML("F:/develop/milad/mujoco/model/box_scene.xml", NULL, error, MAX_MJ_ERROR_LENGTH);
        m_m = mj_loadXML("/home/ecoumans/Downloads/box_scene.xml", NULL, error, MAX_MJ_ERROR_LENGTH);
        
        m_d = mj_makeData(m_m);
        mj_forward(m_m, m_d);
        mjv_makeScene(m_m, &m_scn, 1024);

        //mjr_makeContext(m_m, &m_con, 1);//50*(m_settings.font+1));

        // clear perturbation state
        m_pert.active = 0;
        m_pert.select = 0;
        m_pert.skinselect = -1;

        mjv_defaultOption( &m_vopt);
        // align and scale view, update scene
        //alignscale();
        mjv_updateScene(m_m, m_d, &m_vopt, &m_pert, &m_cam, mjCAT_ALL, &m_scn);
        mjv_addGeoms(m_m, m_d, &m_vopt, &m_pert, mjCAT_ALL, &m_scn);
#if 0
         // clear geoms and add all categories
  scn->ngeom = 0;
  mjv_addGeoms(m, d, opt, pert, catmask, scn);

  // add lights
  mjv_makeLights(m, d, scn);

  // update camera
  mjv_updateCamera(m, d, cam, scn);

  // update skins
  if (opt->flags[mjVIS_SKIN]) {
    mjv_updateSkin(m, d, scn);
  }
#endif


        //if (m) 
        //if (d) {
        //  sim->Load(m, d, filename);
        //  mj_forward(m, d);



        // It is important that all USD stage reads/writes happen from the main thread:
        // https ://graphics.pixar.com/usd/release/api/_usd__page__multi_threading.html
        if (!m_stage)
        {
            return;
        }

        constexpr int numPrimsToCreate = 9;
        const float rotationIncrement = 360.0f / (numPrimsToCreate - 1);
        for (int i = 0; i < numPrimsToCreate; ++i)
        {
            // Create a cube prim.
            const PXR_NS::SdfPath primPath("/World/example_prim_" + std::to_string(i));
            if (m_stage->GetPrimAtPath(primPath))
            {
                // A prim already exists at this path.
                continue;
            }
            PXR_NS::UsdPrim prim = m_stage->DefinePrim(primPath, PXR_NS::TfToken("Cube"));

            //// Set the size of the cube prim.
            //const double cubeSize = 0.5 / PXR_NS::UsdGeomGetStageMetersPerUnit(m_stage);
            //prim.CreateAttribute(PXR_NS::TfToken("size"), PXR_NS::SdfValueTypeNames->Double).Set(cubeSize);

            // Leave the first prim at the origin and position the rest in a circle surrounding it.
            
            m_primsWithRotationOps.push_back({ prim });
            PXR_NS::UsdGeomXformable xformable = PXR_NS::UsdGeomXformable(m_primsWithRotationOps[i].m_prim);
            xformable.ClearXformOpOrder();
            
            auto transform_op = xformable.AddTransformOp();
            m_xformOps.push_back(transform_op);

            //auto scale_op = xformable.AddScaleOp();
            xformable.AddScaleOp(pxr::UsdGeomXformOp::PrecisionDouble).Set(pxr::GfVec3d(100.*0.15, 100.*0.1, 100.*0.05));


        }

        // Subscribe to timeline events so we know when to start or stop animating the prims.
        if (omni::timeline::ITimeline* timeline = omni::timeline::getTimeline())
        {
            m_timelineEventsSubscription = carb::events::createSubscriptionToPop(
                timeline->getTimelineEventStream(),
                [this](carb::events::IEvent* timelineEvent) {
                onTimelineEvent(static_cast<omni::timeline::TimelineEventType>(timelineEvent->type));
            });
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

        // Remove all prims.
        for (auto& primWithRotationOps : m_primsWithRotationOps)
        {
            m_stage->RemovePrim(primWithRotationOps.m_prim.GetPath());
        }
        m_primsWithRotationOps.clear();
        m_xformOps.clear();
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
            m_usdNoticeListenerKey = PXR_NS::TfNotice::Register(PXR_NS::TfCreateWeakPtr(this), &ExampleCppUsdExtension::onObjectsChanged);
        }
    }

    void onObjectsChanged(const PXR_NS::UsdNotice::ObjectsChanged& objectsChanged)
    {
        // Check whether any of the prims we created have been (potentially) invalidated.
        // This may be too broad a check, but handles prims being removed from the stage.
        for (auto& primWithRotationOps : m_primsWithRotationOps)
        {
            if (!primWithRotationOps.m_invalid &&
                objectsChanged.ResyncedObject(primWithRotationOps.m_prim))
            {
                primWithRotationOps.m_invalid = true;
            }
        }
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


        mj_step(m_m, m_d);
        mjv_updateScene(m_m, m_d, &m_vopt, &m_pert, &m_cam, mjCAT_ALL, &m_scn);

        // get geom pointer
        //thisgeom = scn->geoms + i;


        //glTranslatef(geom->pos[0], geom->pos[1], geom->pos[2]);
        //glMultMatrixf(mat);

        //assert(false);

        

        // Update the value of each local and global rotation operation to (crudely) animate the prims around the origin.
        const int numPrims = (int) m_primsWithRotationOps.size();
        if (m_scn.ngeom != numPrims)
        {
            //printf("warning: m_scn->ngeom (%d) != numPrims (%d)\n",(int)m_scn.ngeom, (int)numPrims);
        }
        const float initialLocalRotationIncrement = 360.0f / (numPrims - 1); // Ignore the first prim at the origin.
        const float initialGlobalRotationIncrement = 360.0f / (numPrims - 1); // Ignore the first prim at the origin.
        const float currentAnimTime = omni::timeline::getTimeline()->getCurrentTime() * m_stage->GetTimeCodesPerSecond();
        for (int i = 0; i < numPrims; ++i) // Ignore the first prim at the origin.
        {
            if (m_primsWithRotationOps[i].m_invalid)
            {
                continue;
            }
            PXR_NS::UsdGeomXformable xformable = PXR_NS::UsdGeomXformable(m_primsWithRotationOps[i].m_prim);
            auto& transform = m_xformOps[i];//xformable.AddTransformOp();

            auto mat = PXR_NS::GfMatrix4d();
            mat.SetIdentity();

            int geom_index = (int)i+1;//skip ground plane for now
            auto pos = 100.*PXR_NS::GfVec3d(m_scn.geoms[geom_index].pos[0],m_scn.geoms[geom_index].pos[1],m_scn.geoms[geom_index].pos[2]);
            
            double v[9] = {(double)m_scn.geoms[geom_index].mat[0],(double)m_scn.geoms[geom_index].mat[1],(double)m_scn.geoms[geom_index].mat[2],
                (double)m_scn.geoms[geom_index].mat[3],(double)m_scn.geoms[geom_index].mat[4],(double)m_scn.geoms[geom_index].mat[5],
                (double)m_scn.geoms[geom_index].mat[6],(double)m_scn.geoms[geom_index].mat[7],(double)m_scn.geoms[geom_index].mat[8]};
            auto rot = PXR_NS::GfMatrix3d(
                v[0],v[3],v[6],
                v[1],v[4],v[7],
                v[2],v[5],v[8]);
                    
            
            mat.SetTranslateOnly(pos);
            mat.SetRotateOnly(rot);
            
            transform.Set(mat);
        }
    }

private:

    //mujoco data
    mjvScene m_scn;
    mjrContext m_con;
    mjvPerturb m_pert;
    mjvCamera m_cam;
    mjvOption m_vopt;
    mjModel* m_m;
    mjData* m_d;

    struct PrimWithRotationOps
    {
        PXR_NS::UsdPrim m_prim;
        PXR_NS::UsdGeomXformOp m_localRotationOp;
        PXR_NS::UsdGeomXformOp m_globalRotationOp1;
        bool m_invalid = false;
    };

    PXR_NS::UsdStageRefPtr m_stage;
    PXR_NS::TfNotice::Key m_usdNoticeListenerKey;
    std::vector<PrimWithRotationOps> m_primsWithRotationOps;
    std::vector<PXR_NS::UsdGeomXformOp> m_xformOps;
    std::vector<PXR_NS::UsdGeomXformOp> m_scaleOps;

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
