# ISO 13849 & EU AI Act Annex III Machinery Conformity

`kinetic-guard` provides an architectural compliance bridge between non-deterministic machine learning models and international machinery safety directives.

---

## 1. Regulatory Context

Under the **EU Artificial Intelligence Act (Regulation 2024/1689), Annex III, Section 1**, AI systems that are safety components of machinery subject to the **Machinery Regulation (EU) 2023/1230** are classified as **High-Risk AI Systems**.

They must demonstrate:
1. Continuous risk management.
2. Resilience against faults and environmental inconsistencies.
3. Deterministic safety stops that prevent kinetic harm to operators.

---

## 2. Standards Crosswalk

| Safety Directive | Required Protection Level | KineticGuard Architectural Mechanism |
| :--- | :--- | :--- |
| **ISO 13849-1** | **PL d / PL e (Category 3/4)** | Hardware-in-the-loop CBF-QP projection guaranteeing joint bounds $\le 100\mu\text{s}$. |
| **ISO 10218-1/2** | **Power & Force Limiting (PFL)** | Hard dynamic ceiling on actuator torques $\tau \le \tau_{\max}$. |
| **ISO 10218-1/2** | **Speed & Separation Monitoring (SSM)** | Spatial obstacle barriers dynamically enforcing $h_{\text{obs}}(p) \ge 0$. |
| **EU AI Act** | **Annex III (High-Risk Machinery)** | Cryptographically signed OAX execution certificates proving zero invariant breaches. |

---

## 3. Cryptographic Audit Trail
During operation, KineticGuard logs every projected intervention into an **OpenAssurance Exchange (OAX v1)** certificate signed with an Ed25519 hardware key, establishing an immutable audit record for OSHA and EU notified body inspectors.
