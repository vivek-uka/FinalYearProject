import matplotlib.pyplot as plt

shelfx = [4.73, 4.73, 4.73, 4.73, 4.73, 4.73, 4.4, -0.8, -1.08, -5.79, 0]
shelfy = [-8.66, -6.75, -4.84, -2.93, -1.02, 0.89, 6.67, 9.09, -0.7, -0.95, 0]
shelfa = [3.87, 3.87, 3.87, 3.87, 3.87, 3.87, 4.7, 3.31, 2.9, 2.42, 14]
shelfb = [0.85, 0.85, 0.85, 0.85, 0.85, 0.85, 7.8, 1.76, 15.87, 18, 21.5] 
text = ['Shelf0', 'Shelf1', 'Shelf2', 'Shelf3', 'Shelf4', 'Shelf5', 'Block0', 'Block1', 'Block2', 'Block3', '']
plt.figure(1)
for i in range(len(shelfx)):
    plt.plot([shelfx[i] + shelfa[i]/2, shelfx[i] + shelfa[i]/2, shelfx[i] - shelfa[i]/2, shelfx[i] - shelfa[i]/2, shelfx[i] + shelfa[i]/2], [shelfy[i] + shelfb[i]/2, shelfy[i]-shelfb[i]/2, shelfy[i]-shelfb[i]/2, shelfy[i]+shelfb[i]/2, shelfy[i]+shelfb[i]/2])
    plt.text(shelfx[i], shelfy[i], text[i], horizontalalignment='center',verticalalignment='center')        

plt.xlabel('x in m')
plt.ylabel('y in m')
plt.show()