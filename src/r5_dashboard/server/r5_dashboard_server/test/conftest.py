"""The robot's top speed for the tests. job_manager gets it from r5's config/limits.yaml."""

from r5_dashboard_server import settings

TOP_KMH = 2.16
settings.configure(TOP_KMH)
