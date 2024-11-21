base_clk = 200_000_000
burst = 3_579_545

results = []

for i in range(100):
    clk = base_clk + 1_000_000*i

    desired = burst * 12
    div = desired

    good = (clk*256 + div//2)//div
    actual = clk*256//good
    error = actual - desired
    results.append((error*100.0/desired, clk, actual, desired, error, hex(good)))

    # bad = (clk*256)//div
    # print(hex(bad))
    # actual = clk*256//bad
    # error = actual - desired
    # print(actual, desired, error, error*100.0/desired)

results.sort()
for r in results:
    print(r)
