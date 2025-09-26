---
permalink: /aidx_and_newton_physics
layout: page
title: NVIDIA Newton Physics @ AIDX Lab
---

 <style>
    .video {
        position: relative;
        padding-bottom: 56.25%;
        /* 16:9 */
        height: 0;
    }

    .video img {
        display: block;
        width: 100%;
        height: auto;
        cursor: pointer;
    }

    .video:after {
        content: "";
        position: absolute;
        top: 50%;
        left: 50%;
        width: 64px;
        height: 64px;
        background: url('assets/imgs/play-button.png') no-repeat center center;
        background-size: contain;
        transform: translate(-50%, -50%);
        pointer-events: none;
    }

    .video iframe {
        position: absolute;
        top: 0;
        left: 0;
        width: 100%;
        height: 100%;
    }

    /* image poster clicked, player class added using js */
    .video.player img {
        display: none;
    }

    .video.player:after {
        display: none;
    }
</style>


In this post, we share our first hands-on experiences with [Newton Physics](https://newton-physics.github.io/newton/api/newton.html) — a new, lightweight simulation engine designed to be both easy to use and fast, especially for robotics applications. Although Newton is still in **alpha**, we were curious to see how it performs in one of our most demanding testbeds: in-hand manipulation. Our goal was to evaluate whether Newton could become a serious candidate as our main simulation engine in the future.

**TL;DR**

* Some MuJoCo configurations don’t import automatically into Newton
* Almost everything can already be configured via `newton.solvers.SolverMuJoCo`
* We validated realism by running our [real-world-tested RL controller](https://aidx-lab.org/manipulation/iros24)
* Early training experiments show promising speed and convergence
* Thanks to the simple API, we plan to use Newton for student projects within the scope of our [Advanced Deep Leaning for Robotics course](https://campus.tum.de/tumonline/ee/ui/ca2/app/desktop/#/slc.tm.cp/student/courses/950842341) at the TU Munich.



### Running a real-world-tested RL controller in Newton Physics

<p align="center">
<iframe class="youtube-video" width="746" height="420" src="https://www.youtube.com/embed/OVxJdB_SEBc?start=3" title="YouTube video player" frameborder="0" allow="accelerometer; autoplay; clipboard-write; encrypted-media; gyroscope; picture-in-picture; web-share" allowfullscreen></iframe>
</p>


For us to consider Newton as a main simulation engine, it’s crucial that the **contact dynamics are both highly adjustable and stable**. As a first experiment, we decided to run our [real-world-tested RL controller](https://aidx-lab.org/manipulation/iros24) in a Software-in-the-Loop (SIL) setup.

We already use a SIL setup with IsaacSim (our current primary simulation engine) as the backend. It allows us to test the RL controller script before deploying it on the real robot. Thanks to the simple message-based communication with the robot, the interface is straightforward, and adapting it for Newton was not difficult.

We’ve added the code with annotations below. The main challenges we encountered were:

* **Actuator setup**
* **Friction configuration**

For both, we found simple workarounds (see code). With everything configured correctly, the RL controller can now manipulate the object inside Newton. For us, this demonstrates that Newton already provides the essential features needed to simulate complex in-hand manipulation tasks.


<details><summary>Code</summary>  
<pre><code><small>from itertools import product
import numpy as np
import time
import warp as wp
import newton
from newton.selection import ArticulationView
<details><summary>Packet definition</summary>     from dataclasses import dataclass, field
@dataclass
class CTRL_IN_PACKET:
    counter: int = -1
    q_measured_rad: np.ndarray = field(default_factory=lambda: np.zeros(12))
    q_increment_counter_rad: np.ndarray = field(default_factory=lambda: np.zeros(12))
    q_poti_rad: np.ndarray = field(default_factory=lambda: np.zeros(12))
    dq_increment_radps: np.ndarray = field(default_factory=lambda: np.zeros(12))
    tau_measured_Nm: np.ndarray = field(default_factory=lambda: np.zeros(12))
    PWM: np.ndarray = field(default_factory=lambda: np.zeros(12))
    active: int = 1

@dataclass
class CTRL_OUT_PACKET:
    q_des: np.ndarray
</details>◂ Packet definition

class NewtonSIL:
    def __init__(self, frequency: int = 1000, headless=True, device=None):
        self.headless = headless
        self.device = device
        self.sim_dt = 1.0 / frequency
        self.model_builder_single = newton.ModelBuilder(gravity=0)
        self.model_builder_single.add_mjcf(
            "mj_resources/output/scene.xml", enable_self_collisions=True
        )
<details><summary>Model builder setup</summary>        # Actuators from xml are ignored => set the parameters here
        for i in range(len(self.model_builder_single.joint_dof_mode)):
            self.model_builder_single.joint_dof_mode[i] = (
                newton.JointMode.TARGET_POSITION
            )
            self.model_builder_single.joint_target_ke[i] = 5
            self.model_builder_single.joint_target_kd[i] = 0.3
        # Correct friction, as it is not yet read from the xml
        for i in range(len(self.model_builder_single.shape_key)):
            if self.model_builder_single.shape_key[i] == "object_0/object":
                self.model_builder_single.shape_material_mu[i] = 1
            elif self.model_builder_single.shape_key[i].startswith("hand/"):
                if (
                    "_" in self.model_builder_single.shape_key[i]
                    and int(self.model_builder_single.shape_key[i].split("_")[-1]) % 3
                    == 2
                ):
                    self.model_builder_single.shape_material_mu[i] = 1
                else:
                    self.model_builder_single.shape_material_mu[i] = 0.1
        for i in range(len(self.model_builder_single.body_key)):
            if self.model_builder_single.body_key[i] == "object_0/object":
                self.model_builder_single.body_mass[i] = 0.08
        self.model_builder_single.rigid_contact_torsional_friction = 0.015
        self.model_builder_single.rigid_contact_rolling_friction = 0.0001
        self.model_builder = newton.ModelBuilder(gravity=0)
        self.num_envs = 10
        self.model_builder.replicate(
            self.model_builder_single, self.num_envs, spacing=(1.0, 1.0, 0.0)
        )
</details>◂ Model builder setup
        # Create the final model and solver
        self.model = self.model_builder.finalize()
        self.solver = newton.solvers.SolverMuJoCo(
            self.model,
            use_mujoco_cpu=False,
            integrator="implicitfast",
            cone="elliptic",
            impratio=1.1,
            ncon_per_env=50,
            njmax=500,
        )
<details><summary>Overwrite ignored solver params</summary>        # Disable distal actuators 969154fcd2feb4e094d272da5c79bacfa39971e2
        self.solver.mj_model.actuator_gainprm[3::4] *= 0
        self.solver.mjw_model.actuator_gainprm.assign(
            self.solver.mj_model.actuator_gainprm[None].repeat(self.num_envs, 0)
        )
        self.solver.mj_model.actuator_dynprm[3::4] *= 0
        self.solver.mjw_model.actuator_dynprm.assign(
            self.solver.mj_model.actuator_dynprm
        )  # [None].repeat(self.num_envs, 0))
        self.solver.mj_model.actuator_biasprm[3::4] *= 0
        self.solver.mjw_model.actuator_biasprm.assign(
            self.solver.mj_model.actuator_biasprm[None].repeat(self.num_envs, 0)
        )
        self.solver.mj_model.geom_condim[:] = 4
        self.solver.mjw_model.geom_condim.assign(self.solver.mj_model.geom_condim)
        self.solver.mj_model.eq_solimp[:] = [0.98, 0.999, 0.01745329, 0.5, 2.0]
        self.solver.mjw_model.eq_solimp.assign(self.solver.mj_model.eq_solimp)
        self.solver.mj_model.eq_solref[:] = [0.0021, 1.2]
        self.solver.mjw_model.eq_solref.assign(self.solver.mj_model.eq_solref)
        self.solver.mj_model.dof_frictionloss[:] = 0.001
        self.solver.mjw_model.dof_frictionloss.assign(
            self.solver.mj_model.dof_frictionloss[None].repeat(self.num_envs, 0)
        )
</details>◂ Overwrite ignored solver params
        # Allocate state buffer
        self.state_0 = self.model.state()
        self.state_1 = self.model.state()
        self.control = self.model.control()
<details><summary>Set up articulation views</summary>        # Enumerate joints by names
        fingers = ["ring", "middle", "fore", "thumb"]
        dimensions = ["proximal", "knuckle", "middle"]
        finger_names = [f"hand/{idx}" for idx in fingers]
        joint_names = [
            f"{f_name}_{dim}_joint"
            for (f_name, dim) in product(finger_names, dimensions)
        ]
        assert len(joint_names) == 12
        self.joints_view = ArticulationView(
            self.model,
            "*",
            exclude_joint_types=[newton.JointType.FREE, newton.JointType.DISTANCE],
        )
        self.joints_actuated_view = ArticulationView(
            self.model,
            "*",
            exclude_joints=["*distal*"],
            exclude_joint_types=[newton.JointType.FREE, newton.JointType.DISTANCE],
        )
        self.joint_ctrl_ids = np.array(
            [
                self.joints_actuated_view.joint_dof_names.index(
                    joint_name.split("/")[1]
                )
                for joint_name in joint_names
            ]
        )
        self.object_view = ArticulationView(self.model, "*", include_joints=["object"])
        # goal setting not yet possible
        # self.goal_view = ArticulationView(self.model, "*", include_links=["goal_object"])
        self.default_object_pose = wp.clone(
            self.object_view.get_dof_positions(self.model)
        )
        self.default_object_velocity = wp.clone(
            self.object_view.get_dof_velocities(self.model)
        )
        # self.default_goal_pose = wp.clone(self.goal_view.get_link_transforms(self.model))
        self.initial_dof_positions = wp.clone(
            self.joints_view.get_dof_positions(self.model)
        )
        self.initial_dof_velocities = wp.clone(
            self.joints_view.get_dof_velocities(self.model)
        )
</details>◂ Set up articulation views
        # Open up the GUI and capture the graph of the simulation loop
        if not self.headless:
            self.viewer = newton.viewer.ViewerGL(headless=False)
            self.viewer.set_model(self.model)
            self.start_time = time.time()
            self.sim_time = 0
        self.sim_steps = 0
        with wp.ScopedCapture() as capture:
            # Doing two steps allows us to leave the state swapping inside the graph
            for i in range(2):
                self.state_0.clear_forces()
                self.solver.step(
                    self.state_0, self.state_1, self.control, None, self.sim_dt
                )
                self.state_0, self.state_1 = self.state_1, self.state_0
        self.graph = capture.graph

<details><summary>Helper functions to interact with the simulation</summary>    def reset_object(self, position: np.ndarray = None, rotation: np.ndarray = None):
        self.object_view.set_dof_positions(self.state_0, self.default_object_pose)
        self.object_view.set_dof_velocities(self.state_0, self.default_object_velocity)

    def reset_hand(self):
        self.joints_view.set_dof_positions(self.state_0, self.initial_dof_positions)
        self.joints_view.set_dof_velocities(self.state_0, self.initial_dof_velocities)

    def gravity_on(self):
        self.model.gravity = np.array([0, 0, -9.81])
        if self.device == "cpu":
            self.solver.mj_model.opt.gravity = np.array([0, 0, -9.81])
        elif self.device == "warp":
            self.solver.mjw_model.opt.gravity.assign(np.array([0, 0, -9.81]))

    def gravity_off(self):
        self.model.gravity = np.array([0, 0, 0.0])
        if self.device == "cpu":
            self.solver.mj_model.opt.gravity = np.array([0, 0, 0.0])
        elif self.device == "warp":
            self.solver.mjw_model.opt.gravity.assign(np.array([0, 0, 0.0]))

    def set_goal(self, key: str, goal: np.ndarray):
        # TODO: Vizualize goal object in viewer
        pass
</details>◂ Helper functions to interact with the simulation

    # Pass the simulated robot measurements to the RL controller
    def get_in_pckts(self) -> CTRL_IN_PACKET:
        joint_angles = self.joints_actuated_view.get_attribute(
            "joint_q", self.state_0
        ).numpy()[0, self.joint_ctrl_ids]
        return CTRL_IN_PACKET(
            self.sim_steps,
            joint_angles,
            joint_angles,
            joint_angles,
        )

    # Every time we recieve control targets from the RL controller -> simulate
    def send_out_pckt(self, ctrl_out_pckt: CTRL_OUT_PACKET) -> None:
        ctrl_npy = np.zeros((self.num_envs, 12))
        ctrl_npy[:, self.joint_ctrl_ids] = ctrl_out_pckt.q_des.copy()
        ctrl = wp.array(
            ctrl_npy, dtype=wp.float32, device=self.control.joint_target.device
        )
        self.joints_actuated_view.set_attribute("joint_target", self.control, ctrl)
        wp.capture_launch(self.graph)
        if not self.headless:
            self.viewer.begin_frame(self.sim_time)
            self.viewer.log_state(self.state_0)
            # self.viewer.log_contacts(self.contacts, self.state_0)
            self.viewer.end_frame()
            self.sim_time += self.sim_dt * 2
        self.sim_steps += 1</small></code></pre>
</details>



### Learning with MuJoCo Warp backend works


<p align="center">
<div class="video">
    <img src="assets/imgs/blog/newton_many.jpg">
    <!-- <iframe width="746" height="420" src="https://www.youtube.com/embed/OVxJdB_SEBc?start=24&autoplay=1"  frameborder="0" allowfullscreen></iframe>-->
</div>
</p>

As the next step, we began implementing a simplified version of our full simulation backend. In this setup, we ignored all configurations and left out domain randomization. The main effort was in providing the required states to our learning framework and enabling detailed access for resetting the simulation.

While this took a bit more time to implement, it is now fully functional. Training — despite not yet tuned for maximum speed — already runs at a pace comparable to our IsaacSim-based setup. This is a promising sign that Newton can keep up with our existing workflow while offering a simpler and lighter interface.


### Conclusion

For us, this opens the door to using Newton not only in research but also in teaching contexts, such as for the student projects in the ["Advanced Deep Leaning for Robotics" course](https://campus.tum.de/tumonline/ee/ui/ca2/app/desktop/#/slc.tm.cp/student/courses/950842341) at Technical University of Munich (TUM), where ease of use and speed are key.

<small>This work was carried out by [Dominik Winkelbauer](https://scholar.google.com/citations?user=kduGd8wAAAAJ){:target="_blank"} and [Johannes Pitz](https://www.linkedin.com/in/johannes-pitz/){:target="_blank"} from the AIDX Lab, TUM.</small>


<script>
document.addEventListener("DOMContentLoaded", function() {
    var videos = document.querySelectorAll(".video");

    videos.forEach(function(video) {
        video.addEventListener("click", function() {
            var conts = video.childNodes;
            var le = conts.length;
            var ifr = null;

            for (var i = 0; i < le; i++) {
                if (conts[i].nodeType === 8) { // comment node
                    ifr = conts[i].textContent;
                }
            }

            if (ifr) {
                video.classList.add("player");
                video.innerHTML = ifr;
                video.removeEventListener("click", arguments.callee);
            }
        });
    });
});

</script>
