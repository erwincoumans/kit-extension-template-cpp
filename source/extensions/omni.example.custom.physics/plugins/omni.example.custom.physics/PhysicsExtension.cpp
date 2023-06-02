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

#include "ExportUtils.h"

#include <carb/Defines.h>
#include <carb/PluginUtils.h>

#include <omni/example/custom/physics/ICustomPhysicsInterface.h>
#include <omni/ext/ExtensionsUtils.h>
#include <omni/ext/IExt.h>
#include <omni/kit/IApp.h>
#include <omni/timeline/ITimeline.h>
#include <omni/timeline/TimelineTypes.h>

#include <pxr/base/gf/transform.h>
#include <pxr/usd/usd/notice.h>
#include <pxr/usd/usd/primRange.h>
#include <pxr/usd/usd/stage.h>
#include <pxr/usd/usd/stageCache.h>
#include <pxr/usd/usdGeom/metrics.h>
#include <pxr/usd/usdGeom/xform.h>
#include <pxr/usd/usdUtils/stageCache.h>

#include "pxr/usd/usd/collectionMembershipQuery.h"
#include "pxr/usd/usdGeom/xformCache.h"

#include <omni/kit/IStageUpdate.h>
#include <omni/kit/KitUpdateOrder.h>

#include <omni/physics/schema/IUsdPhysics.h>

#include <vector>


using namespace PXR_NS;
class OmniMuJoCoUpdateNode : public omni::physics::schema::IUsdPhysicsListener,
                             public TfWeakBase,
                             public omni::example::custom::physics::ICustomPhysicsInterface {
    using TransformationCache = std::unordered_map<pxr::SdfPath, std::pair<carb::Float3, carb::Float4>>;

    pxr::UsdStageRefPtr m_stage = nullptr;
    bool m_stageInitialized = false;
    bool m_paused = true;
    omni::example::custom::physics::UsdData m_usdData;
    std::string file_name;
    // mujoco data
    mjvScene m_scn;
    mjrContext m_con;
    mjvPerturb m_pert;
    mjvCamera m_cam;
    mjvOption m_vopt;
    mjModel *m_m;
    mjData *m_d;

    TfNotice::Key m_usdNoticeListenerKey;
    std::vector<UsdGeomXformOp> m_xformOps;
    std::vector<UsdGeomXformOp> m_scaleOps;
    std::vector<GfVec3d> m_scales;
    carb::events::ISubscriptionPtr m_updateEventsSubscription;
    carb::events::ISubscriptionPtr m_timelineEventsSubscription;

  public:
    // CARB_PLUGIN_INTERFACE("omni.example.custom.physics", 1, 0)
    omni::kit::StageUpdateNode *m_stageUpdateNode = nullptr;

    // omni::physics::schema::IUsdPhysicsListener interface
    void parsePrim(const pxr::UsdPrim &prim, omni::physics::schema::ObjectDesc *objectDesc, uint64_t,
                   const pxr::TfTokenVector &appliedApis) override {
        // std::cout << "parsePrim" << std::endl;
    }

    void reportObjectDesc(const pxr::SdfPath &path, const omni::physics::schema::ObjectDesc *objectDesc) override {
        switch (objectDesc->type) {
        case omni::physics::schema::ObjectType::eScene: {
            const pxr::UsdPrim scenePrim = m_stage->GetPrimAtPath(path);
            if (scenePrim) {
                // We process only scenes without any API or with PhysxSceneAPI
                const pxr::TfTokenVector &appliedApis = scenePrim.GetPrimTypeInfo().GetAppliedAPISchemas();
                if (!appliedApis.empty()) {
                    static const pxr::TfToken gCustomPhysicsSceneAPIToken("CustomPhysicsAPI");
                    for (auto &api : appliedApis) {
                        if (api == gCustomPhysicsSceneAPIToken) {
                            m_usdData.sceneDesc = *(const omni::physics::schema::SceneDesc *)objectDesc;
                            m_usdData.scenePath = path;
                        }
                    }
                }
            }
        } break;
        case omni::physics::schema::ObjectType::eRigidBody: {
            const omni::physics::schema::RigidBodyDesc *inDesc =
                (const omni::physics::schema::RigidBodyDesc *)objectDesc;
            for (auto &ownerPath : inDesc->simulationOwners) {
                if (ownerPath == m_usdData.scenePath || true) {
                    m_usdData.rigidBodyMap[path] = inDesc;
                    m_usdData.bodyPrims.push_back(m_stage->GetPrimAtPath(path));
                    // todo: convert USD data to MuJoCo / URDF. Once all is done, 'compile' the model
                    // todo2: refactor MuJoCo to not use a global 'Model' and 'Data' but more fine-grained data that can
                    // be incrementally updated.
                }
            }
        } break;

        case omni::physics::schema::ObjectType::eSphereShape:
        case omni::physics::schema::ObjectType::eCubeShape:
        case omni::physics::schema::ObjectType::eCapsuleShape:
        case omni::physics::schema::ObjectType::eCylinderShape:
        case omni::physics::schema::ObjectType::eConeShape:
        case omni::physics::schema::ObjectType::eMeshShape:
        case omni::physics::schema::ObjectType::eCustomShape: {
            const omni::physics::schema::ShapeDesc *inDesc = (const omni::physics::schema::ShapeDesc *)objectDesc;
            m_usdData.shapeMap[path] = inDesc;
            m_usdData.shapePrims.push_back(m_stage->GetPrimAtPath(path));

            pxr::UsdPrim prim = m_stage->GetPrimAtPath(path);
            std::cout << "reportObjectDesc shape " << prim.GetName().GetText() << std::endl;
            pxr::GfVec3f scale;
            prim.GetAttribute(TfToken("xformOp:scale")).Get(&scale);
            m_scales.push_back(scale);
        } break;
        }
    }

    // StageUpdateNodeDesc implementation
    void CARB_ABI onAttach(long int stageId, double metersPerUnit) {
        std::cout << "OmniMuJoCoInterface::onAttach" << std::endl;
        m_stage = pxr::UsdUtilsStageCache::Get().Find(pxr::UsdStageCache::Id::FromLongInt(stageId));
    }
    static void CARB_ABI onAttach(long int stageId, double metersPerUnit, void *userData) {
        assert(usedData);
        OmniMuJoCoUpdateNode *node = (OmniMuJoCoUpdateNode *)userData;
        node->onAttach(stageId, metersPerUnit);
    }

    // detach the stage
    void CARB_ABI onDetach() {
        std::cout << "OmniMuJoCoInterface::onDetach" << std::endl;
        m_stage = nullptr;
    }
    // detach the stage
    static void CARB_ABI onDetach(void *userData) {
        assert(usedData);
        OmniMuJoCoUpdateNode *node = (OmniMuJoCoUpdateNode *)userData;
        std::cout << "static void CARB_ABI onDetach" << std::endl;
        node->onDetach();
    }

    // simulation was paused
    void CARB_ABI onPause() {
        std::cout << "OmniMuJoCoInterface::onPause" << std::endl;
        m_paused = true;
    }

    // simulation was paused
    static void CARB_ABI onPause(void *userData) {
        assert(usedData);
        OmniMuJoCoUpdateNode *node = (OmniMuJoCoUpdateNode *)userData;
        node->onPause();
    }

    // simulation was stopped(reset)
    void CARB_ABI onStop() {
        std::cout << "OmniMuJoCoInterface::onStop" << std::endl;
        reset();
    }

    // simulation was stopped(reset)
    static void CARB_ABI onStop(void *userData) {
        assert(usedData);
        OmniMuJoCoUpdateNode *node = (OmniMuJoCoUpdateNode *)userData;
        node->onStop();
    }

    void CARB_ABI convertAndInitializeMujoco() {

#if _WIN32
        file_name = "C:\\Projects\\Repos\\kit-ext\\box_scene";
#else
        file_name = "/home/ecoumans/Downloads/box_scene";
#endif
        std::cout << "exporting to urdf" << std::endl;
        omni::example::custom::physics::ExportUtils exporter(m_stage, m_usdData);
        exporter.ExportToUrdf(file_name + ".urdf");
        exporter.ExportToMjcf(file_name + ".xml");
        mjv_defaultScene(&m_scn);

        char error[MAX_MJ_ERROR_LENGTH] = "";

        m_m = mj_loadXML((file_name + ".xml").c_str(), NULL, error, MAX_MJ_ERROR_LENGTH);
        if (m_m) {
            m_d = mj_makeData(m_m);
            mj_forward(m_m, m_d);
            mjv_makeScene(m_m, &m_scn, 1024);

            // mjr_makeContext(m_m, &m_con, 1);//50*(m_settings.font+1));

            // clear perturbation state
            m_pert.active = 0;
            m_pert.select = 0;
            m_pert.skinselect = -1;

            mjv_defaultOption(&m_vopt);
            // align and scale view, update scene
            // alignscale();
            mjv_updateScene(m_m, m_d, &m_vopt, &m_pert, &m_cam, mjCAT_ALL, &m_scn);
            mjv_addGeoms(m_m, m_d, &m_vopt, &m_pert, mjCAT_ALL, &m_scn);
        } else {
            std::cout << "Couldn't load " << file_name << std::endl;
        }
        const int numPrims = (int)m_usdData.bodyPrims.size();
        printf("m_scn->ngeom (%d), numPrims (%d)\n", (int)m_scn.ngeom, (int)numPrims);
        for (int i = 0; i < numPrims; ++i) {
            UsdGeomXformable xformable = UsdGeomXformable(m_usdData.bodyPrims[i]);
            xformable.ClearXformOpOrder();

            auto transform_op = xformable.AddTransformOp(pxr::UsdGeomXformOp::PrecisionDouble);
            m_xformOps.push_back(transform_op);

            if (m_usdData.shapeMap.find(m_usdData.bodyPrims[i].GetPath()) != m_usdData.shapeMap.end()) {
                auto scale = m_usdData.shapeMap[m_usdData.bodyPrims[i].GetPath()]->localScale;
                std::cout << "prim scale " << scale << std::endl;
                // xformable.AddScaleOp(pxr::UsdGeomXformOp::PrecisionDouble).Set(scale);
            }
        }
    }
    // simulation was resumed
    void CARB_ABI onResume(float currentTime) {
        std::cout << "OmniMuJoCoInterface::onResume" << std::endl;

        if (!m_stageInitialized) {
            reset();
            omni::physics::schema::IUsdPhysics *usdPhysics =
                carb::getFramework()->acquireInterface<omni::physics::schema::IUsdPhysics>();
            if (usdPhysics) {
                pxr::UsdGeomXformCache xfCache;
                usdPhysics->registerPhysicsListener(this);
                pxr::UsdPrimRange range = m_stage->Traverse(pxr::UsdTraverseInstanceProxies());
                omni::physics::schema::PrimIteratorRange primIteratorRange(range);
                // The following parse all the usd prims, connect to all registered listeners and call their
                // reportObjectDesc methods with the pair of <path, primDescriptor> for each prim type.
                // the listener is responsible to decide what to do with the incoming data
                // OmniMuJoCoUpdateNode::reportObjectDesc is currently digest the incoming physics scene and rigid
                // bodies. It will capture the physics scene with CustomPhysicsAPI apiSchema as the custom physicsScene
                // It will keep a record of the rigid bodies that have this custom physics simulators; i.e., all those
                // with the simulationOwner rel = physicsScene path this m_usdData.scenePath simulation owner
                usdPhysics->loadFromRange(m_stage, xfCache, primIteratorRange);
                usdPhysics->unregisterPhysicsListener(this);

                if (m_usdData.scenePath != pxr::SdfPath()) {
                    //"mSceneDesc.gravityDirection"
                    std::cout << "mSceneDesc.gravityMagnitude=" << m_usdData.sceneDesc.gravityMagnitude << std::endl;
                }
                m_stageInitialized = true;
                convertAndInitializeMujoco();
            } else {
                std::cout << "Error: couldn't find IUSdPhysics interface. Is omni.usdphysics enabled?" << std::endl;
            }
        }
    }
    // simulation was resumed
    static void CARB_ABI onResume(float currentTime, void *userData) {
        assert(usedData);
        OmniMuJoCoUpdateNode *node = (OmniMuJoCoUpdateNode *)userData;
        node->onResume(currentTime);
    }

    // this gets called in every update cycle of Kit (typically at each render frame)
    void CARB_ABI onUpdate(float currentTime, float elapsedSecs, const omni::kit::StageUpdateSettings *settings) {
        if (!m_stageInitialized)
            return;
        mj_step(m_m, m_d);
        mjv_updateScene(m_m, m_d, &m_vopt, &m_pert, &m_cam, mjCAT_ALL, &m_scn);
        double metersPerUnit = PXR_NS::UsdGeomGetStageMetersPerUnit(m_stage);

        // Update the value of each local and global rotation operation to (crudely) animate the prims around the
        // origin.
        const int numPrims = (int)m_usdData.bodyPrims.size();
        // if (m_scn.ngeom != numPrims) {
        //     printf("warning: m_scn->ngeom (%d) != numPrims (%d)\n", (int)m_scn.ngeom, (int)numPrims);
        // }
        std::unique_ptr<SdfChangeBlock> changeBlock;

        for (int i = 0; i < numPrims; ++i) // Ignore the first prim at the origin.
        {

            auto &prim = m_usdData.bodyPrims[i];
            UsdGeomXformable xformable = UsdGeomXformable(prim);
            xformable.ClearXformOpOrder();

            auto &transform = m_xformOps[i]; // xformable.AddTransformOp();

            GfVec3d scale(1.0);
            auto mat = GfMatrix4d();
            mat.SetIdentity();
            int geom_index = i; // skip ground plane for now
            GfVec3d pos =
                1.0 / metersPerUnit *
                GfVec3d(m_scn.geoms[geom_index].pos[0], m_scn.geoms[geom_index].pos[1], m_scn.geoms[geom_index].pos[2]);

            double v[9] = {(double)m_scn.geoms[geom_index].mat[0], (double)m_scn.geoms[geom_index].mat[1],
                           (double)m_scn.geoms[geom_index].mat[2], (double)m_scn.geoms[geom_index].mat[3],
                           (double)m_scn.geoms[geom_index].mat[4], (double)m_scn.geoms[geom_index].mat[5],
                           (double)m_scn.geoms[geom_index].mat[6], (double)m_scn.geoms[geom_index].mat[7],
                           (double)m_scn.geoms[geom_index].mat[8]};
            auto rot = GfMatrix3d(v[0], v[3], v[6], v[1], v[4], v[7], v[2], v[5], v[8]);
            // mat.SetScale(scale);
            mat.SetTranslateOnly(pos);
            mat.SetRotateOnly(rot);
            // transform.Set(mat);
            // std::cout << " prim " << i << " transfrom" << mat << "scale " << scale << std::endl;
            pxr::UsdTimeCode time = pxr::UsdTimeCode::Default();
            xformable.AddTransformOp(pxr::UsdGeomXformOp::PrecisionDouble).Set(mat);

            // if (m_usdData.shapeMap.find(prim.GetPath()) != m_usdData.shapeMap.end()) {
            //     scale = m_usdData.shapeMap[prim.GetPath()]->localScale;
            //     xformable.AddScaleOp(pxr::UsdGeomXformOp::PrecisionDouble).Set(scale);
            // }

        }
    }

    static void CARB_ABI onUpdate(float currentTime, float elapsedSecs, const omni::kit::StageUpdateSettings *settings,
                                  void *userData) {
        assert(usedData);
        OmniMuJoCoUpdateNode *node = (OmniMuJoCoUpdateNode *)userData;
        node->onUpdate(currentTime, elapsedSecs, settings);
    }

    /*
        The following call backs are called when there is change in usd scene (e.g. user interaction, scripting, )
        Each plugin should query the usd stage based on provided prim path name and sync changes related to their
    */
    // this gets called when a new Usd prim was added to the scene
    void CARB_ABI onPrimAdd(const pxr::SdfPath &primPath) {
        std::cout << "OmniMuJoCoInterface::onPrimAdd" << std::endl;
    }

    static void CARB_ABI onPrimAdd(const pxr::SdfPath &primPath, void *userData) {
        assert(usedData);
        OmniMuJoCoUpdateNode *node = (OmniMuJoCoUpdateNode *)userData;
        node->onPrimAdd(primPath);
    }

    // this gets called when some properties in the usd prim was changed (e.g. manipulator or script changes transform)
    void CARB_ABI onPrimOrPropertyChange(const pxr::SdfPath &primOrPropertyPath) {
        std::cout << "OmniMuJoCoInterface::onPrimOrPropertyChange" << std::endl;
    }
    // this gets called when some properties in the usd prim was changed (e.g. manipulator or script changes transform)
    static void CARB_ABI onPrimOrPropertyChange(const pxr::SdfPath &primOrPropertyPath, void *userData) {
        assert(usedData);
        OmniMuJoCoUpdateNode *node = (OmniMuJoCoUpdateNode *)userData;
        node->onPrimOrPropertyChange(primOrPropertyPath);
    }

    // this gets called when the named usd prim was removed from the scene
    void CARB_ABI onPrimRemove(const pxr::SdfPath &primPath) {
        std::cout << "OmniMuJoCoInterface::onPrimRemove" << std::endl;
    }
    // this gets called when the named usd prim was removed from the scene
    static void CARB_ABI onPrimRemove(const pxr::SdfPath &primPath, void *userData) {
        assert(usedData);
        OmniMuJoCoUpdateNode *node = (OmniMuJoCoUpdateNode *)userData;
        node->onPrimRemove(primPath);
    }

    /**
     * Temporary raycast handler.  This will become part of a more general user event handler.
     *
     * @param orig the ray origin.  Set to NULL to send a stop command for grabbing.
     * @param dir the ray direction (should be normalized).
     * @param input whether the input control is set or reset (e.g. mouse down).
     */
    void CARB_ABI onRaycast(const float *orig, const float *dir, bool input) {
        std::cout << "OmniMuJoCoInterface::onRaycast" << std::endl;
    }
    static void CARB_ABI onRaycast(const float *orig, const float *dir, bool input, void *userData) {
        assert(usedData);
        OmniMuJoCoUpdateNode *node = (OmniMuJoCoUpdateNode *)userData;
        node->onRaycast(orig, dir, input);
    }

    // specific reset to this simulator
    void reset() {
        m_paused = true;
        m_stageInitialized = false;

        m_usdData.bodyPrims.clear();
        m_usdData.scenePath = pxr::SdfPath();
    }

    void onDefaultUsdStageChanged(long stageId) override {
        TfNotice::Revoke(m_usdNoticeListenerKey);
        m_timelineEventsSubscription = nullptr;
        m_updateEventsSubscription = nullptr;
        m_stage.Reset();

        if (stageId) {
            m_stage = UsdUtilsStageCache::Get().Find(UsdStageCache::Id::FromLongInt(stageId));
            m_usdNoticeListenerKey =
                TfNotice::Register(TfCreateWeakPtr(this), &OmniMuJoCoUpdateNode::onObjectsChanged);
        }
    }

    void onObjectsChanged(const UsdNotice::ObjectsChanged &objectsChanged) {
        // Check whether any of the prims we created have been (potentially) invalidated.
        // This may be too broad a check, but handles prims being removed from the stage.
    }

    void onTimelineEvent(omni::timeline::TimelineEventType timelineEventType) {
        switch (timelineEventType) {
        case omni::timeline::TimelineEventType::ePlay: {
            // startAnimatingPrims();
        } break;
        case omni::timeline::TimelineEventType::eStop: {
            // stopAnimatingPrims();
        } break;
        default: {

        } break;
        }
    }
};


OmniMuJoCoUpdateNode *gOmniMuJoCoUpdateNode = nullptr;

CARB_EXPORT void carbOnPluginStartup() {
    std::cout << "carbOnPluginStartup" << std::endl;
    CARB_LOG_WARN("carbOnPluginStartup physics.");
    carb::Framework *framework = carb::getFramework();
    auto iStageUpdate = framework->tryAcquireInterface<omni::kit::IStageUpdate>();
    {
        omni::kit::StageUpdateNodeDesc desc = {0};
        desc.displayName = "Custom Physics";
        desc.order = omni::kit::update::eIUsdStageUpdatePhysics;
        desc.onAttach = OmniMuJoCoUpdateNode::onAttach;
        gOmniMuJoCoUpdateNode = new OmniMuJoCoUpdateNode();
        desc.userData = gOmniMuJoCoUpdateNode;
        desc.onDetach = OmniMuJoCoUpdateNode::onDetach;
        desc.onUpdate = OmniMuJoCoUpdateNode::onUpdate;
        desc.onResume = OmniMuJoCoUpdateNode::onResume;
        desc.onPause = OmniMuJoCoUpdateNode::onPause;
        desc.onStop = OmniMuJoCoUpdateNode::onStop;
        desc.onRaycast = nullptr;

        gOmniMuJoCoUpdateNode->m_stageUpdateNode = iStageUpdate->createStageUpdateNode(desc);
    }
}

CARB_EXPORT void carbOnPluginShutdown() {
    carb::Framework *framework = carb::getFramework();
    auto iStageUpdate = framework->tryAcquireInterface<omni::kit::IStageUpdate>();

    if (gOmniMuJoCoUpdateNode != nullptr) {
        iStageUpdate->destroyStageUpdateNode(gOmniMuJoCoUpdateNode->m_stageUpdateNode);
        delete gOmniMuJoCoUpdateNode;
        gOmniMuJoCoUpdateNode = nullptr;
    }
}

const struct carb::PluginImplDesc pluginImplDesc = {"omni.example.custom.physics.plugin", "An example C++ extension.",
                                                    "NVIDIA", carb::PluginHotReload::eEnabled, "dev"};

CARB_PLUGIN_IMPL(pluginImplDesc, OmniMuJoCoUpdateNode)
CARB_PLUGIN_IMPL_DEPS(omni::physics::schema::IUsdPhysics)
void fillInterface(OmniMuJoCoUpdateNode &iface) {}

// CARB_PLUGIN_IMPL(pluginImplDesc, ICustomPhysicsInterface)

// CARB_PLUGIN_IMPL(pluginImplDesc, omni::example::custom::physics::OmniMuJoCoUpdateNode)
// CARB_PLUGIN_IMPL_DEPS(omni::physics::schema::IUsdPhysics)
// void fillInterface(omni::example::custom::physics::OmniMuJoCoUpdateNode &iface) {}