const initList = async () => {
  folderAry = await window.currentProject.queryModel([window.BimEngine.ModelType.FOLDER])

    // README: 
    // 1. folderAry 必须按照楼层由低到高的顺序排列  
    // folderAry 也可以通过 guid 数组，获取对应的模型
    folderAry.sort((a, b) => {
      const aArr = a.name.split("#")
      const bArr = b.name.split("#")

      const aName = aArr[1]
      const bName = bArr[1]
      
      const basementRegex = /^地下室-B(\d+)层$/
      const buildingRegex = /^建筑T1-(\d{2})层$/
      
      const aBasement = basementRegex.exec(aName)
      const bBasement = basementRegex.exec(bName)
      const aBuilding = buildingRegex.exec(aName)
      const bBuilding = buildingRegex.exec(bName)

      // 如果两者都是地下室楼层，则按从低到高排序（B5 最低）
      if (aBasement && bBasement) {
        return parseInt(bBasement[1]) - parseInt(aBasement[1])
      }

      // 如果一个是地下室，一个是建筑楼层，则地下室优先
      if (aBasement && !bBasement) {
        return -1
      }
      if (!aBasement && bBasement) {
        return 1
      }

      // 如果两者都是建筑楼层，则按楼层数字从低到高排序
      if (aBuilding && bBuilding) {
        return parseInt(aBuilding[1]) - parseInt(bBuilding[1])
      }

      return 0
    })
    // console.log(folderAry)

    modList.value = []
    folderAry.map((item) => {
      let text = getStringBeforeSubstring(item.name, '#')
      if(!modList.value.includes(text)) {
        modList.value.push(text)
        buildings.push({ name: text, data: [], controller: null })
      }
    })
    // modList.value = folderAry
    initCheckBoxAll()

    buildings.map((item) => {
      folderAry.map((folder) => {
        let text = getStringBeforeSubstring(folder.name, '#')
        if(item.name == text) {
          item.data.push({
            folder: folder, 
            M_original: BimEngine.Matrix4.clone(folder.getWorldMatrix()),
            M_explode: BimEngine.Matrix4.clone(BimEngine.Matrix4.IDENTITY),
            M_extract: BimEngine.Matrix4.clone(BimEngine.Matrix4.IDENTITY)
          })
        }
      })

      // console.log(offsetX.value, offsetY.value, offsetZ.value)
      // let tt = `[-100, 0, 0]`
      // tt = JSON.parse(tt);
      // let tt2 = [-100, 0, 0]
      // console.log(typeof(tt))
      // console.log(typeof(tt2))

      // const moveOffset = new Array(offsetX.value, offsetY.value, offsetZ.value)
      // item.controller = createExplosionAndExtractController({
      //   folderItems: item.data,
      //   distanceFactor: distanceFactor.value,
      //   pulloutOffset: moveOffset,
      //   viewer: window.viewer
      // })

      // item.controller.enableExtract()
    })

    const moveOffset = new Array(offsetX.value, offsetY.value, offsetZ.value)
    const itemController = createExplosionAndExtractController({
        folderItems: buildings,
        distanceFactor: distanceFactor.value,
        pulloutOffset: moveOffset,
        viewer: window.viewer
      })

    itemController.enableExtract()
}