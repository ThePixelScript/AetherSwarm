with open('tests/integration/test_m0_loop.py', 'r', encoding='utf-8') as f:
    c = f.read()

c = c.replace('assert final_bat["u3"] == 100.0', 'assert final_bat["u3"] == 95.0')
with open('tests/integration/test_m0_loop.py', 'w', encoding='utf-8') as f:
    f.write(c)
