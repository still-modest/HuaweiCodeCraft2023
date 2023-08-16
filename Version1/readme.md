最初始版本，使用最原始的贪心算法，优先合成7类工作台，让其快速生产，4.5.6类工作台根据7类工作台的需要进行生产。

根据需要购买：先判断7类工作台是否有产品，如果有则送去8类或9类工作台，如果没有则判断缺少哪一类产品，
例如缺少6号原材料，判断6类工作台是否有产品，如果有则送去7类工作台，如果没有则判断缺少哪一类产品，以此类推。

问题：一开始对工作台的判断顺序为根据工作台序号，但这样会使机器人向偏远的工作台购买以及取货物，浪费较多时间，
解决方法：对最高级别到最低级别的工作台构建成一个树，根据每一级的工作台寻找最近距离的下一级别工作台，以此来使机器人行驶较短的距离而节省时间。


问题：工作台每一帧都会给机器人发送消息，这样会导致机器人没有执行完上一个步骤就被下达另一个命令
解决办法：将工作台和机器人进行绑定，当机器人还没有执行完上一个指令时，不会再接收其他工作台发送的指令。

问题：工作台给查询机器人的工作状态并给机器人发命令的顺序是按序号的，这导致工作台可能会给较远的0号机器人发命令，而不给较近的1号机器人发信号
解决办法：工作台先计算所有没有任务的机器人的距离，对最近的机器人发命令。

