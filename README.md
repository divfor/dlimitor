# dlimitor
a distributed limitor design for multiple NUMAs system.
<img width="959" alt="Pasted Graphic 13" src="https://github.com/user-attachments/assets/603340af-cc3f-488d-8884-5722963f6827">

# usage example
```c
    uint32_t i, thread_num = 16;
    uint32_t dlnum = (1U << 0);
    dlimitor_cfg_t cfg = {
        .sliding_power_w = 4,
        .second_ticks = get_hz(),
        .update_interval = get_hz()/((2<<4)-1),
        .flags = (LOCAL_UPDATE), // LOCAL_UPDATE | PTR_BUCKET
    };
    uint32_t t = 19, q = t - 5;
    dlimitor_qos_t default_qos = {
        .qos_num = 4,
        .limit_total = (1U<<t)/dlnum,            /* total limit up to 2048 Kpps */
        .limits = { [0] = 3*2*(1U<<q)/dlnum,     /* white-list, limit to 3*128=384 Kpps */
                    [1] = 3*3*(1U<<q)/dlnum,     /* new-incoming, limit to 3*192=576 Kpps */
                    [2] = 3*3*(1U<<q)/dlnum,     /* established, limit to 3*192=576 Kpps */
                    [3] = 1*24*(1U<<q)/dlnum,   /* grey-list, limit to 1*24*64=1536 Kpps */
		   }
    };
    if ((dlnum = dlarray_init(&g_da, dlnum, thread_num, &cfg, &default_qos)) < 0)
        goto ERR_RETURN;
    for (i = 0; i < dlnum; i++) {
        if (dlarray_add_limitor(&g_da, i, NULL) < 0)
            goto ERR_RETURN;
    }
    printf("Successful add limitor [%d]\n", g_da.dlimitor_count);
```

# algorithm

Exponentially-weighted sliding mean and variance
```c
alpha = 1 - ((1 - alpha)^(duration / period))
diff = x - mean
incr = alpha * diff
mean = mean + incr
variance = (1 - alpha) * (variance + diff * incr)
```

pass_probability = p, drop_probability = 1 - p
```c
rxps * p = expected_limit_rate  # rxps - arrival number per second, p - probability for pass
```

Calculate the remaining limit capacity
```c
limit_allocated = min(limits[qos], limit_remaining) 
limit_remaining -= min(limits[qos], txps) # txps - passed number per second
```

Calculate p
```c
expected_p = limits[qos] / rxps; # Constraint formula rxps * p = limit for pass probability p
actual_p = txps / rxps; # measure actual rate from sliding move average
excess =  actual_p > expected_p ? (actual_p - expected_p) : 0; # how many percent actually exceeded
dispatch_p = expected_p - (excess << 4) # rapid fix and send out p, only used for next interval
```

# Design:
1. Each worker increments its own exclusive counter (exclusive cacheline), and all operations are triggered by packet arrivals, without setting up additional timer. 
2. The summation of all worker counters is not operated by a dedicated aggregation server, but by a atomic-cas winner worker right after each interval of summation.
3. At each summation time, the winner worker calculates the sliding rx/tx rates and the forward probabilities, and actively pushed to the all workers immediately.
5. If no packets arrive, this causes the update to the timerless mechanism described above to stall, but then there is no need to overrun the limiter action.
6. If the limiter instances grow to hundreds of thousands, the performance impact of the above actions depends only on the frequency of atomic cas lock memory bus. For example, 100,000 dlimitors with 30 atomic cas per socket per second is roughly 3 million cas operations per second, which is still relatively easy.
