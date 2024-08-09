# dlimitor
a distributed limitor design for multiple NUMAs system.
<img width="959" alt="Pasted Graphic 13" src="https://github.com/user-attachments/assets/603340af-cc3f-488d-8884-5722963f6827">


# algorithm

Exponentially-weighted sliding mean and variance
```c
diff = x - mean
incr = alpha * diff
mean = mean + incr
variance = (1 - alpha) * (variance + diff * incr)
```

pass_probability = p, drop_probability = 1 - p
```c
arrive_rate * p = expected_limit_rate  
```

Calculate the remaining limit capacity
```c
limits[n] = min(limits[n], limit_remaining) 
limit_remaining -= min(limits[n], txps)  
```

Calculate p
```c
exceed = txps - limits[n] # actual excess
fix_limit = limits[n] - exceed # fix back
rxps * p = fix_limit # Constraint formula for pass probability p
p = fixed_limit / rxps # get p, only used for next interval
```

# Design:
1. Each worker increments its own exclusive counter (exclusive cacheline), and all operations are triggered by packet arrivals, without setting up additional timer. 
2. The summation of all worker counters is not operated by a dedicated aggregation server, but by a atomic-cas winner worker right after each interval of summation.
3. At each summation time, the winner worker calculates the sliding rx/tx rates and the forward probabilities, and actively pushed to the all workers immediately.
5. If no packets arrive, this causes the update to the timerless mechanism described above to stall, but then there is no need to overrun the limiter action.
6. If the limiter instances grow to hundreds of thousands, the performance impact of the above actions depends only on the frequency of atomic cas lock memory bus. For example, 100,000 dlimitors with 30 atomic cas per socket per second is roughly 3 million cas operations per second, which is still relatively easy.
