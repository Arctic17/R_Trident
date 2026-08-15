\# Developer Profile \& Communication Preferences



\## Who I Am

\* I am an engineering student developing an independent, high-performance robotics project with guidance from university faculty.

\* My goal is to demonstrate industrial-grade mechatronics and systems engineering competency to professors and future employers.

\* I operate across multiple disciplines: mechanical design, electrical cabinet layout, real-time Linux environments, and C/Python programming.



\## How I Like Things Answered

\* \*\*Tone \& Level:\*\* Treat me like a capable systems engineer. Skip the basic tutorials and conversational fluff; give me direct, high-level, and highly technical answers.

\* \*\*Formatting:\*\* Provide all code, configurations, and documentation inside fully formatted Markdown code blocks so I can copy and paste them directly into my workspace.

\* \*\*Engineering Rigor:\*\* When discussing design iterations, frame them as "Before \& After" engineering trade-offs. Ground your answers in physical realities (e.g., thermal buoyancy, EMI shielding, rotational inertia).

\* \*\*Actionable Directives:\*\* Do not give vague advice. If there is an error, give me the exact CLI commands, C-code fixes, or XML memory mappings required to solve it.



\## Project Context: R\_Trident

\* \*\*System:\*\* A custom, high-speed direct-drive Rotary Delta Robot (parallel kinematics).

\* \*\*Control Host:\*\* Raspberry Pi 5 / Compute Module 5 (CM5) running a PREEMPT\_RT Linux kernel and LinuxCNC 2.9.8.

\* \*\*Communication Bus:\*\* Real-time EtherCAT (IgH Master) running a 1 ms cyclic task loop via `eth0` MAC binding.

\* \*\*Actuation:\*\* Direct-drive setup using three STEPPERONLINE 400W AC Servo Motors (A6M60-400H2B1-M17) and A6-EC Servo Drives via the CiA 402 protocol.

\* \*\*CAD Software:\*\* Autodesk Inventor 2025.

\* \*\*Kinematics:\*\* 150 mm bicep, 335 mm forearm, 80 mm base pivot radius, and 35 mm end-effector radius.

\* \*\*Mechanical Structure:\*\* Optimized for minimum mass using carbon fiber tube spars (20.7 mm OD / 18.2 mm ID) and Nylon-GF double-lap structural epoxy joints.

\* \*\*Software Stack:\*\* Uses custom real-time C components (`halcompile`) to manage an industrial PackML/IEC state machine and logical interlocks.



