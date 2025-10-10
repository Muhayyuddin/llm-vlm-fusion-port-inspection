"""
VLM Inspection Package

This package provides Vision-Language Model (VLM) based inspection and surveillance capabilities
for maritime port monitoring and UAV-based inspection systems.

Available Models:
- Florence-2: Comprehensive VLM for detailed scene understanding
- BLIP2: Efficient VLM for image captioning and VQA
- T5: Text processing and surveillance analysis
- nanoVLM: Ultra-compact VLM (~222M parameters) for efficient captioning & VQA
- SmolVLM: Minimal footprint VLM (256M/500M parameters) with strong multimodal capability

Surveillance Nodes:
- surveillance_node: Florence-2 based surveillance
- blip2_surveillance_node: BLIP2 based surveillance  
- t5_surveillance_node: T5 enhanced surveillance
- nanovlm_surveillance_node: Ultra-efficient nanoVLM surveillance
- smolvlm_surveillance_node: Advanced SmolVLM surveillance

Services:
- florence_service: Florence-2 model service
- blip2_service: BLIP2 model service
- t5_service: T5 model service
- nanovlm_service: nanoVLM model service
- smolvlm_service: SmolVLM model service
"""

__version__ = "1.0.0"
__author__ = "VLM Inspection Team"

# Import main services
from .florence_service import FlorenceService
from .blip2_service import BLIP2Service
from .t5_service import T5Service
from .nanovlm_service import NanoVLMService
from .smolvlm_service import SmolVLMService

# Import surveillance nodes
from .surveillance_node import SurveillanceNode
from .blip2_surveillance_node import BLIP2SurveillanceNode
from .t5_surveillance_node import T5SurveillanceNode
from .nanovlm_surveillance_node import NanoVLMSurveillanceNode
from .smolvlm_surveillance_node import SmolVLMSurveillanceNode

__all__ = [
    'FlorenceService',
    'BLIP2Service', 
    'T5Service',
    'NanoVLMService',
    'SmolVLMService',
    'SurveillanceNode',
    'BLIP2SurveillanceNode',
    'T5SurveillanceNode',
    'NanoVLMSurveillanceNode',
    'SmolVLMSurveillanceNode'
]