# multi-platform-relocation
relocation-operations-calculation module for multi-platform BSS operations
## API usage
Refer to ``test_main.py`` in ``\tests`` for examples.

```python
from main import get_relocation_routes

result = get_relocation_routes(**case)

# various results are returned in variale result

# van location
loc = result['loc'][van_id][t_step]
# load/unload quantity
n_r = result['n_r'][van_id][t_step]
# destinations
dest = result['destination']
# distance left for each van
van_dis_left = result['van_dis_left']
# objective
obj = result['objective']
```