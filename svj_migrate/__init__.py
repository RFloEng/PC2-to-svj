"""The SVJ standard's suspension migration (corner section -> bodies, joints, elements).
Vendored from RFloEng/SVJ-standard-vehicle-json tools/ (Apache-2.0); regenerate with vendor_svj_migrate.py."""
from .migrate_suspension import migrate

__all__ = ["migrate"]
