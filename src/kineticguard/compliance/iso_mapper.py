"""
Machinery Safety and Regulatory Mapping for KineticGuard.
Crosswalks Control Barrier Function guarantees to ISO 13849, ISO 10218, and EU AI Act Annex III.
"""

from typing import Dict, Any

MACHINERY_STANDARDS = {
    "iso_13849": {
        "standard_name": "ISO 13849-1: Safety of Machinery - Safety-related parts of control systems",
        "target_level": "Performance Level d / e (PL d/e, Category 3/4)",
        "clauses": {
            "ISO-13849-5.1": {
                "title": "Safety functions (Monitored Stop & Speed/Torque Limiting)",
                "mechanism": "Control Barrier Functions enforcing joint velocity and position bounds in hardware loop"
            },
            "ISO-13849-5.2": {
                "title": "Diagnostic Coverage & Fault Detection",
                "mechanism": "Microsecond heartbeat watchdog with automatic C2 continuous emergency braking"
            }
        }
    },
    "iso_10218": {
        "standard_name": "ISO 10218-1 / ISO 10218-2: Industrial & Collaborative Robots",
        "target_level": "Power and Force Limiting (PFL) & Speed and Separation Monitoring (SSM)",
        "clauses": {
            "ISO-10218-5.4": {
                "title": "Speed and Separation Monitoring (SSM)",
                "mechanism": "Obstacle barrier functions dynamically computing minimum Euclidean clearance polyhedra"
            },
            "ISO-10218-5.5": {
                "title": "Power and Force Limiting (PFL)",
                "mechanism": "Rigid torque ceilings tau <= tau_max projected via convex QP filter"
            }
        }
    },
    "eu_ai_act_annex_iii": {
        "standard_name": "EU Artificial Intelligence Act (Annex III: High-Risk Machinery Component)",
        "target_level": "Harmonized Machinery Conformity",
        "clauses": {
            "EU-AI-ANNEX-III-1": {
                "title": "Safety Components in High-Risk Industrial Machinery",
                "mechanism": "Deterministic mathematical proof ensuring probabilistic VLA actions never breach physical envelopes"
            }
        }
    }
}
